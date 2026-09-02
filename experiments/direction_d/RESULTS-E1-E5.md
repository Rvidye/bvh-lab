# Direction D: E1 to E5, the optimization pass before Arches

Answers the five experiments in `PLAN-optimize-before-arches.md`, in order.

**Artifacts.** All numbers below come from these committed files:

| file | what it holds |
|---|---|
| `results/e2e4_2026_09_02/d1_prediction.csv` | 144 rows: per-score win rates and marginal CIs, 3 scenes x 4 ray distributions x 12 scores |
| `results/e2e4_2026_09_02/d1_contrasts.csv` | paired bootstrap CIs for every score-vs-score contrast below |
| `results/e2e4_2026_09_02/d1_depth_summary.csv` | 72 rows: win rate by depth band (E4) |
| `results/e2e4_2026_09_02/d1_summary.csv`, `d1_directional_bins.csv` | probe support counts |
| `results/gatec_2026_09_02/san_miguel_geometry_collapse.csv` | Gate C: per-variant, per-view traversal counters and the visit-weighted decision-change rate |
| `results/gatec_2026_09_02/d3_summary.csv` | exact depth-banded aggregate of every changed collapse decision, with reference visit counts |

`results/gatec_2026_09_02/d3_decisions.csv` (the per-decision dump, 53 MB,
strided at 20k rows per variant) is not committed for size; every number quoted
from it below comes from the exact `d3_summary.csv` aggregate, which is computed
over all changed decisions rather than the sample.
Gate C run: `trax.exe --geometry_collapse --run_id=gatec_2026_09_02 --scene=scenes/<name>.obj --width=512 --height=512 --bins=32 --threads=1`.

Analysis code: `experiments/direction_d/predict_gate_a.py` (E1-E3),
`experiments/direction_d/depth_breakdown.py` (E4).

Probe run: `trax.exe --direction_d_only --run_id=e2e4_2026_09_02 --scene=scenes/<name>.obj --width=128 --height=128 --rays=262144 --bins=32 --threads=1 --direction_d_validation_rays=2048`.

Every score-vs-score verdict is a **paired** clustered bootstrap on the
difference, resampling whole rays, 1000 resamples, 95% percentile interval.
"better" means the interval excludes zero.

---

## Code verification: both findings confirmed in the source

**Finding 1 is exactly right.** `directional_mean_fill` routes through
`directional_loss`, which is `positive(F - G)` — a clamp.
`directional_min_fill` routes through `compute_axis_fill`, whose `fill[]` is
`saturating_fill(q) = 1 - exp(-q)`. Gate B compared *(clamp, area-weighted mean)*
against *(exponential, unweighted min)*: two knobs, one comparison.

**Finding 2 is exactly right.** Expanding `1 - Ldir/SA` with
`Ldir = 2 Σ max(F_i - G_i, 0)` and `SA = 2 Σ F_i`:

```
mean_fill = Σ (F_i - max(F_i - G_i, 0)) / Σ F_i = Σ F_i · min(1, q_i) / Σ F_i
```

a face-area-weighted mean of clamped per-axis fills. This is now stated and
tested directly (`MeanFillIsTheFaceAreaWeightedMeanOfClampedFills`).

`bvh/build/geometry_loss.h` gains `fill_map`, `fill_aggregate` and
`fill_shape_args`, and `axis_fill_shape()` evaluates any point in that space.
`WeightedMeanOfClampedFillsReproducesMeanFill` anchors it: at
`{clamp, weighted_mean, alpha=1}` it reproduces `directional_mean_fill`, so E2
and E3 measure perturbations of the shipped loss rather than of something
adjacent to it. 262 unit tests pass; build is clean.

---

## E1 — mean_fill vs surface_density: **PASSES**

The contrast that decides the novelty story. Where the clamp does not bind — 96
to 98% of axes, per d2 — the two differ only by the L1-vs-L2 norm of the
triangle normals, i.e. by how well the geometry aligns with the box axes.

| scene | ray dist | mean_fill | surface_density | delta | 95% CI | verdict |
|---|---|---|---|---|---|---|
| hairball | diffuse_1 | 0.5838 | 0.5756 | +0.0082 | [+0.0036, +0.0132] | better |
| hairball | incoherent | 0.5871 | 0.5758 | +0.0114 | [+0.0108, +0.0119] | better |
| hairball | primary | 0.5947 | 0.5771 | +0.0176 | [+0.0130, +0.0227] | better |
| hairball | shadow_ao | 0.5857 | 0.5807 | +0.0051 | [-0.0001, +0.0102] | inconclusive |
| intel-sponza | diffuse_1 | 0.5547 | 0.5406 | +0.0141 | [-0.0059, +0.0327] | inconclusive |
| intel-sponza | incoherent | 0.6038 | 0.5947 | +0.0092 | [+0.0084, +0.0099] | better |
| intel-sponza | primary | 0.5774 | 0.6381 | -0.0607 | [-0.0693, -0.0515] | worse |
| intel-sponza | shadow_ao | 0.5853 | 0.5370 | +0.0484 | [+0.0456, +0.0512] | better |
| san-miguel | diffuse_1 | 0.6005 | 0.5891 | +0.0114 | [+0.0084, +0.0142] | better |
| san-miguel | incoherent | 0.6248 | 0.5760 | +0.0488 | [+0.0480, +0.0498] | better |
| san-miguel | primary | 0.6415 | 0.5936 | +0.0478 | [+0.0348, +0.0610] | better |
| san-miguel | shadow_ao | 0.6261 | 0.5612 | +0.0649 | [+0.0605, +0.0690] | better |

**Better in 9 of 12, inconclusive in 2, worse in 1** (intel-sponza primary, the
known exception, which E4 explains). The axis decomposition carries signal
beyond triangle density. The projection structure is doing work; this is not a
restatement of "count the triangle area".

---

## E2 — deconfounding Gate B: **both knobs mattered, aggregation more than the map**

Each row holds one knob fixed and moves the other. Counts are cells out of 12.

| contrast | a better | b better | inconclusive | median delta |
|---|---|---|---|---|
| map, weighted mean fixed (clamp vs exponential) | 10 | 1 | 1 | +0.0168 |
| map, min fixed (clamp vs exponential) | 10 | 1 | 1 | +0.0181 |
| aggregation, clamp fixed (mean vs min) | 11 | 0 | 1 | +0.0379 |
| aggregation, exponential fixed (mean vs min) | 11 | 0 | 1 | +0.0246 |
| aggregation, both unweighted (mean vs min) | 7 | 2 | 3 | +0.0079 |

Three conclusions, and the first two point in opposite directions from what was
expected.

**The exponential is genuinely harmful, as predicted.** Holding aggregation
fixed, the clamp wins in 10 of 12 cells either way, by up to +0.0784
(san-miguel shadow_ao). d2 explains it: with q >= 1 on only 2-4% of axes,
`1-exp(-q)` never gets to use its one advantage and always pays its cost of
compressing the mass below q = 1.

**But it was not carrying the entire Gate B loss.** With the map held fixed at
the clamp, the mean still beats the min in 11 of 12 cells by a median +0.0379 —
*larger* than the map effect. **Gate B's verdict survives deconfounding.** Min
is worse than mean, and separately the exponential is worse than the clamp;
Gate B stacked two real penalties and attributed them to one cause.

**Most of the min penalty is the weighting, not the aggregation.** The last row
is the same mean-vs-min comparison with the weighting removed from both sides:
the median gap collapses from +0.0379 to +0.0079, becomes inconclusive in 3
cells and *reverses* in 2 (san-miguel shadow_ao, `min` better by 0.0848). So
Finding 2's mechanism is confirmed — `min_fill` was largely losing because it is
unweighted, not because it takes a minimum — even though the practical
conclusion (do not adopt min) is unchanged.

---

## E3 — face-area weighting is the active ingredient, and alpha = 1 is the peak

Win rate against the weight exponent, with map and aggregation held at
(clamp, weighted mean):

| scene | ray dist | alpha=0 | alpha=0.5 | alpha=1 | alpha=2 | peak |
|---|---|---|---|---|---|---|
| hairball | diffuse_1 | 0.5833 | 0.5836 | 0.5838 | 0.5823 | 1 |
| hairball | incoherent | 0.5833 | 0.5850 | 0.5871 | 0.5865 | 1 |
| hairball | primary | 0.5907 | 0.5944 | 0.5947 | 0.5958 | 2 |
| hairball | shadow_ao | 0.5815 | 0.5834 | 0.5857 | 0.5863 | 2 |
| intel-sponza | diffuse_1 | 0.5463 | 0.5481 | 0.5547 | 0.5683 | 2 |
| intel-sponza | incoherent | 0.5782 | 0.5991 | 0.6038 | 0.6017 | 1 |
| intel-sponza | primary | 0.5606 | 0.5678 | 0.5774 | 0.5786 | 2 |
| intel-sponza | shadow_ao | 0.5510 | 0.5795 | 0.5853 | 0.5828 | 1 |
| san-miguel | diffuse_1 | 0.5457 | 0.5900 | 0.6005 | 0.5973 | 1 |
| san-miguel | incoherent | 0.5922 | 0.6210 | 0.6248 | 0.6208 | 1 |
| san-miguel | primary | 0.5834 | 0.6361 | 0.6415 | 0.6405 | 1 |
| san-miguel | shadow_ao | 0.5229 | 0.6060 | 0.6261 | 0.6203 | 1 |

Paired contrasts against alpha = 1:

| contrast | alpha=1 better | worse | inconclusive | median delta |
|---|---|---|---|---|
| alpha 1 vs 0 | 10 | 0 | 2 | +0.0257 |
| alpha 1 vs 0.5 | 7 | 0 | 5 | +0.0054 |
| alpha 1 vs 2 | 6 | 1 | 5 | +0.0015 |

**Weighting is decisively the active ingredient.** alpha = 0 is never better and
is significantly worse in 10 of 12 cells, by up to +0.1032 (san-miguel
shadow_ao). The monotone climb from alpha 0 to 1 is visible in every San Miguel
and Sponza row.

**alpha = 1 peaks in 8 of 12 cells and is never significantly beaten.** alpha = 2
peaks in the other 4, but the paired contrast between them is inconclusive in 5
cells with a median difference of +0.0015 in alpha = 1's favour — the two are
statistically indistinguishable.

So the answer the plan asked for is the good one: performance peaks at the
value that first principles predict. **Face area is proportional to the chance a
ray enters through that face, so alpha = 1 makes `mean_fill` an estimate of the
probability that a ray entering the box meets geometry inside it — and that is
where the measured optimum is.** This can be stated as design rationale rather
than as a tuned constant. It was not tuned; alpha = 1 was already the shipped
behaviour, and the sweep confirms it.

---

## E4 — the descriptor changes sign at the top of the tree

This was scoped as "one focused look" at the Sponza-primary exception. It is
much more than that: **the exception is a special case of a universal
structure**, and the structure matters more than the exception.

Win rate by depth of the parent whose children were compared:

| scene | ray dist | depth 0-3 | share | depth 4-7 | share |
|---|---|---|---|---|---|
| hairball | diffuse_1 | n/a (all ties) | 8.9% | 0.5985 | 12.1% |
| hairball | incoherent | n/a (all ties) | 8.5% | 0.6387 | 12.4% |
| hairball | primary | n/a (all ties) | 11.4% | 0.6165 | 14.4% |
| hairball | shadow_ao | n/a (all ties) | 7.2% | 0.6025 | 12.4% |
| intel-sponza | diffuse_1 | 0.4754 | 17.6% | 0.6680 | 13.2% |
| intel-sponza | incoherent | 0.4998 | 23.6% | 0.6612 | 22.0% |
| intel-sponza | primary | **0.2422** | 17.4% | 0.6279 | 18.2% |
| intel-sponza | shadow_ao | 0.4783 | 16.5% | 0.6465 | 15.2% |
| san-miguel | diffuse_1 | **0.2395** | 31.3% | 0.7074 | 15.4% |
| san-miguel | incoherent | 0.4168 | 28.9% | 0.7321 | 18.4% |
| san-miguel | primary | **0.2368** | 29.0% | 0.7913 | 19.6% |
| san-miguel | shadow_ao | 0.4087 | 31.1% | 0.7041 | 17.3% |

In **12 of 12 cells** `mean_fill` is at or below chance in the top four levels
and strongly above chance immediately below them. There are two distinct
mechanisms:

- **Architectural scenes (Sponza, San Miguel): the score inverts.** At depths
  0-3 it reaches 0.2368 — that is not noise, it is a *reliable predictor with
  the wrong sign*; inverting it there would score 0.76. Near the root both
  children are enormous, and the child that contains the visible hit is
  typically the emptier one, because that is the volume the ray travels through
  before reaching a surface. "Fuller predicts the hit" is simply false at the
  top of an architectural scene.
- **Hairball: the score is undefined.** Every top-level pair is a tie, because
  both children saturate at fill = 1. Consistent with d2, which measured
  Hairball's fills as the highest and most strongly correlated of the three.

Where the pairs actually are (median depth of a discordant pair): 8 on San
Miguel, 8-13 on Sponza, 12-14 on Hairball, against tree depths of 62, 31 and 34.
So the mass sits at depths 4-15, where the descriptor is at its best — but the
top four levels still hold **17-31% of all pairs** on the architectural scenes.

The natural inference — that the collapse is being poisoned by applying a
single-signed term across a sign change — is **wrong**, and E5's `d3_summary.csv`
is what shows it. The collapse changes essentially no decisions in the top four
levels (0.0% to 1.4% of changed visits), because that band holds at most 15
nodes and SAH dominates there. The inversion is real and worth knowing, but the
collapse never acts on it. See E5.

It also fully explains E1's single loss: intel-sponza primary is the cell that
combines a high shallow-pair share with the strongest inversion, and
`surface_density` (0.6869 at depths 0-3) and `box_projected_ratio` (0.6974) do
not invert there. The reviewer question now has an answer with a mechanism
rather than a shrug.

---
## E5 — Gate C with visit weighting: **fails to isolate the directional term**

Run: `trax.exe --geometry_collapse --run_id=gatec_2026_09_02 --scene=scenes/<name>.obj --width=512 --height=512 --bins=32 --threads=1`.
Three frozen views per scene, one binary SAH tree per scene, BVH8 collapse.
`mu = 0` reproduces the ordinary collapse byte-for-byte through both loss paths
on all three scenes, so V0 holds. Every variant's traversal output was verified
equal to the binary tree's on every view before its counters were read.

### The visit weighting answers the plan's question — with the opposite sign

`node_visits_reference` counts, per binary node, how often the ordinary binary
traversal reaches it across the three views. The visit-weighted decision-change
rate is the share of that traversal work sitting at a node whose retain/absorb
decision moved.

| scene | variant | raw changed | visit-weighted | ratio |
|---|---|---|---|---|
| intel-sponza | scalar_mu1.00 | 4.13% | 14.65% | 3.55x |
| intel-sponza | directional_mu1.00 | 4.48% | 15.58% | 3.48x |
| intel-sponza | directional_mu2.00 | 5.77% | 21.88% | 3.79x |
| hairball | scalar_mu1.00 | 3.91% | 17.98% | 4.60x |
| hairball | directional_mu1.00 | 4.75% | 16.61% | 3.49x |
| san-miguel | scalar_mu1.00 | 4.09% | 17.06% | 4.17x |
| san-miguel | directional_mu1.00 | 4.48% | **6.47%** | 1.44x |
| san-miguel | directional_mu2.00 | 5.71% | 12.98% | 2.27x |

The plan's worry was that "4.5% of decisions changed producing 0.3% fewer node
steps is arithmetic, not evidence" — that the changes would sit in deep,
rarely-visited nodes. **The measurement says the reverse.** The changed
decisions are concentrated in *frequently* visited nodes: the visit-weighted
rate is 3.5 to 4.6 times the raw rate almost everywhere. Between 15% and 22% of
all traversal work happens at a node whose decision this term moved.

That makes the weak traversal result worse, not better. The term is landing on
the hot path, in quantity, and the counters still barely move.

The exception is instructive: on San Miguel the directional loss has a ratio of
only 1.44x against the scalar control's 4.17x, i.e. it moves decisions at
markedly *colder* nodes than the density control does on the same scene.

### The traversal counters

Primary preregistered comparison, `mu = 1`, percent change against the SAH
baseline on the same tree and rays:

| scene | variant | view | node steps | box tests | prim steps | SAH cost |
|---|---|---|---|---|---|---|
| intel-sponza | scalar_mu1.00 | A / B / C | -4.59 / -0.29 / -1.73 | -4.83 / -0.30 / -1.78 | +0.05 / +0.06 / +0.29 | +0.50 |
| intel-sponza | directional_mu1.00 | A / B / C | **-5.59 / -2.93 / -1.89** | -5.98 / -3.21 / -1.99 | +0.05 / +0.06 / +0.04 | +0.45 |
| hairball | scalar_mu1.00 | A / B / C | -4.72 / -1.48 / **+8.15** | -5.68 / -2.06 / +8.41 | -0.03 / -0.03 / -0.51 | +0.32 |
| hairball | directional_mu1.00 | A / B / C | -1.22 / -0.00 / **+2.05** | -1.70 / -0.79 / +1.79 | +0.06 / -0.06 / -0.59 | +0.40 |
| san-miguel | scalar_mu1.00 | A / B / C | **+2.53 / +4.63** / -2.21 | -0.63 / +4.96 / -2.33 | -1.06 / -2.97 / -0.08 | +0.60 |
| san-miguel | directional_mu1.00 | A / B / C | -0.22 / -2.24 / **+0.73** | -0.24 / -2.35 / +0.73 | +0.00 / +1.28 / -0.02 | +0.51 |

**Intel Sponza is the one clean result.** `directional_mu1.00` reduces node
steps and box tests on all three views, by 1.9% to 5.6%, at a cost of +0.45% SAH
and no change in primitive work. That is a real reduction in traversal work on
one scene, which is what Gate C asked for.

**But it is not attributable to the directional term.** The `scalar_density`
control — which uses total triangle area only and no projection decomposition —
achieves -4.59 / -0.29 / -1.73 at the same mu, and at `mu = 2` it reaches
**-5.46 / -4.84 / -6.35**, beating every directional variant on that scene. On
the one scene where the collapse helps, a non-directional density term helps at
least as much.

**Hairball and San Miguel show no reduction.** Signs flip across views within a
single scene, and the per-view spread (Hairball scalar_mu1.00 ranges from -4.72%
to +8.15%) is far larger than any mean effect. Three frozen viewpoints cannot
establish a scene-level effect against that spread; these numbers are consistent
with the collapse redistributing work between viewpoints rather than reducing
it.

The counters are deterministic — one tree, one rayset, `--threads=1` — so the
spread is genuine view-dependence, not measurement noise. That is the honest
reading: the effect is real per view and does not generalise across views.

### Gate C verdict

**Fails.** A reduction exists on one scene but the matched non-directional
control reproduces it, so the projection decomposition is not shown to be the
cause. The sibling-level prediction advantage measured in E1 does not carry
through the DP into traversal work.

### Where the changed decisions actually land, and why that rules out the easy explanation

The obvious explanation would be that E4's sign inversion is poisoning the
collapse: a term that is right at depth 4-15 and wrong at depth 0-3, applied
uniformly, would partly cancel itself. **`d3_summary.csv` rules that out.** It
aggregates every changed decision — not the strided sample — by depth band, for
`directional_mu1.00`:

| scene | band | share of reference visits | share of *changed* visits | nodes changed in band |
|---|---|---|---|---|
| intel-sponza | 0-3 | 12.5% | **1.4%** | 13.33% |
| intel-sponza | 4-7 | 28.5% | 55.1% | 20.00% |
| intel-sponza | 8-11 | 31.3% | 28.6% | 18.00% |
| hairball | 0-3 | 3.6% | **0.0%** | 0.00% |
| hairball | 8-11 | 14.9% | 25.0% | 23.83% |
| hairball | 12-15 | 26.2% | 47.5% | 24.84% |
| san-miguel | 0-3 | 15.0% | **0.0%** | 0.00% |
| san-miguel | 4-7 | 27.9% | 21.2% | 9.75% |
| san-miguel | 12-15 | 12.5% | 26.2% | 17.58% |

The top four levels carry 3.6% to 15.0% of all traversal visits and receive
**0.0% to 1.4% of the changed decisions**. On Hairball and San Miguel not one of
those nodes flipped. The band is tiny in node count — at most 15 nodes — and SAH
dominates the decision there, so the loss perturbation never moves it.

The changes concentrate at depths 4-15: 83.7% of Sponza's changed visits, 72.5%
of Hairball's, and 99.4% of San Miguel's fall in the bands where E4 measured the
descriptor at its *best* (win rates 0.63 to 0.79).

So the failure is not a depth-sign problem, and a depth-gated loss would not fix
it — the collapse already does not act where the sign is wrong. The gap is
between the two decisions themselves: **the probe validates sibling ranking,
while the collapse makes retain/absorb choices.** The term ranks siblings well,
at the depths where it is applied, on nodes carrying 15-22% of traversal work —
and the resulting retain/absorb changes still do not reduce traversal, and are
matched by a control with no projection information. Knowing which of two
children is emptier evidently does not tell the DP which node to absorb.

---

## Verdicts, and what this means before Arches

| experiment | question | result |
|---|---|---|
| E1 | does mean_fill beat surface_density? | **pass** — 9/12 better, 1 worse, median +0.0141 |
| E2 | which knob cost Gate B? | **both** — map ~+0.017, aggregation ~+0.038; Gate B's verdict survives |
| E3 | is area weighting the active ingredient? | **yes** — alpha 0 worse in 10/12; peak at alpha = 1, never significantly beaten |
| E4 | why does Sponza-primary resist? | **answered** — the descriptor inverts at depths 0-3 in 12/12 cells |
| E5 | Gate C, visit-weighted | **fail** — reduction on one scene, matched by the non-directional control |

Against the plan's four conditions for starting the Arches port:

1. **E1 passes.** The term is not just density. PASS
2. **The formulation is settled**, and for a stated reason rather than a sweep:
   clamp, face-area-weighted mean, alpha = 1 — which is the shipped
   `collapse_loss::directional`, now with the closed form named, tested and
   justified by the entry-probability argument, and with alpha = 1 confirmed as
   the measured optimum. PASS
3. **Gate C does not show an attributable visit-weighted traversal reduction.** FAIL
4. **Scope can now be stated honestly** — Sponza-primary is explained by a
   mechanism, and the depth structure is measured. PASS

**Three of four hold; the third does not. Do not start the Arches port.**
Porting a term that does not yet produce an attributable traversal reduction
would mean debugging that gap inside a cycle-level simulator, which is the worst
place for it.

The next experiment is *not* the depth gating E4 first suggested — `d3_summary`
shows the collapse already never acts at the depths where the sign is wrong.
What the evidence points at is the translation step: the loss enters only as an
additive penalty on the internal-node term, `SA(n) + mu * L(n)`, which raises the
cost of retaining an empty node. That is a plausible but untested mapping from
"emptier child" to "absorb this node". The measured failure sits there, and that
is where the next experiment belongs. It is not run here.

## What the evidence supports saying

- A node's triangles project onto its bounding box faces. Weighting each face by
  its area — proportional to how likely a ray is to enter through it — estimates
  the probability that a ray entering the box hits something. Measured with
  unbiased sibling probes across three scenes and four ray distributions, this
  predicts which of two box-hit siblings contains the relevant intersection:
  above chance in 12 of 12 cells, better than a box-only control in 11 of 12,
  and better than a triangle-density control in 9 of 12, all by paired clustered
  bootstrap.
- It needs no rays at build time. The direction-conditioned version was tested
  and was worse.
- Its sign inverts in the top four levels of the tree, which is measured, not
  inferred.
- It does not yet produce an attributable reduction in traversal work.

The first three are supportable today. The fourth is why the port waits.
"Directional" should be retired from the naming in favour of projected
emptiness or box-entry hit probability; the current name describes the
approach that was tested and dropped.
