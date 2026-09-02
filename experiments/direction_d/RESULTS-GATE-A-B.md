# Direction D: Gate A, Gate B, and the per-node loss terms

**Work packages:** WP-A (prediction and Gate A), WP-B (`directional_min` and the
saturating map), WP-C d2 (per-node loss terms).

**Artifacts every number below comes from:**

| file | what it holds |
|---|---|
| `results/gateb_2026_09_02/d1_prediction.csv` | 84 rows: per-score win rates and marginal CIs, 3 scenes x 4 ray distributions x 7 scores |
| `results/gateb_2026_09_02/d1_contrasts.csv` | 96 rows: paired bootstrap CIs for score-vs-score differences |
| `results/d1_large_sampled_2026_08_31/d1_prediction.csv` | 80 rows: the same for the earlier 4-scene run (adds Bistro, has 5 scores) |
| `results/d1_large_sampled_2026_08_31/d1_contrasts.csv` | 48 rows |
| `results/nodeterms_2026_09_02/d2_node_summary.csv` | per-scene saturation rates, q quantiles, fill correlations |

Analysis code: `experiments/direction_d/predict_gate_a.py`.
Probe run `gateb_2026_09_02` and node-term run `nodeterms_2026_09_02` were both
produced by this worktree; both record `git_commit=6097937`, `dirty=1`, because
the WP-B loss variants were uncommitted when they ran.

The per-node dump `d2_node_terms.csv` (595,260 sampled rows, 139 MB, 47 MB
gzipped) is **not** committed -- it is too large for the repository. Every d2
statistic quoted below is computed over *all* 33.2M nodes, not over that
sample, and lives in the committed `d2_node_summary.csv`. To regenerate the
dump and the summary:

```
trax.exe --node_terms --run_id=nodeterms_2026_09_02 --scene=scenes/<name>.obj --bins=32 --threads=1 --git_commit=6097937 --dirty=1
```

and the probe run behind Gate A/B:

```
trax.exe --direction_d_only --run_id=gateb_2026_09_02 --scene=scenes/<name>.obj --width=128 --height=128 --rays=262144 --bins=32 --threads=1 --direction_d_validation_rays=2048 --git_commit=6097937 --dirty=1
```

Summary in one line: **Gate A passes, Gate B fails, and the d2 measurements
falsify the diagnosis that motivated WP-B.**

---

## 0. What changed in the analysis, and why it matters

Two corrections to the method were needed before any gate could be read.

**Ties.** `d1_summary.csv` reports `accuracy`, which keeps ties in the
denominator. A score that declines to separate two siblings is not wrong; it is
silent. `win_rate = correct / (correct + incorrect)` excludes ties and reports
them separately. This is the single change that turned the earlier apparent null
result into a measurable effect. Both statistics are in `d1_prediction.csv` so
they cannot be confused.

**Paired inference.** The earlier Gate A read "beats the control" off two point
estimates. That is not a test. Every score-vs-score claim below now comes from a
paired clustered bootstrap that resamples the same rays for both scores and
forms a CI on the *difference* (`d1_contrasts.csv`, `pairing=paired`). A
difference counts only when its 95% interval excludes zero.

The bootstrap resamples whole rays (1000 resamples, percentile interval). Rays
that produced no discordant pair are real clusters contributing (0, 0); they are
reconstructed from `rays - rays_with_pairs` and included in the population. The
resample vector is drawn from its exact multinomial law by the sequential
conditional-binomial method rather than one ray at a time -- with k up to 5.2e5
per cell, the naive draw is not tractable in Python and was the reason this
analysis had not been run.

Tightening the test made Gate A *harder*, not easier: comparing point estimates
passes cells that a paired interval leaves inconclusive. The criterion that
matters most, beating `box_projected_ratio`, holds either way.

---

## 1. Gate A -- does the directional descriptor predict at all? **PASS**

Run `gateb_2026_09_02`, 3 scenes x 4 ray distributions. `directional` is the
probe's per-direction fill ratio `geom_proj(g,d) / box_proj(box,d)` along the
actual ray direction.

| scene | ray dist | directional | 95% CI | surf_density | prim_count | box_proj | >0.5 | >density | >box_proj (paired) |
|---|---|---|---|---|---|---|---|---|---|
| hairball | diffuse_1 | 0.5829 | [0.5769, 0.5891] | 0.5756 | 0.5636 | 0.5108 | yes | yes | yes |
| hairball | incoherent | 0.5835 | [0.5829, 0.5842] | 0.5758 | 0.5754 | 0.5192 | yes | yes | yes |
| hairball | primary | 0.5853 | [0.5790, 0.5913] | 0.5771 | 0.5742 | 0.5162 | yes | yes | yes |
| hairball | shadow_ao | 0.5859 | [0.5790, 0.5933] | 0.5807 | 0.5672 | 0.5136 | yes | yes | yes |
| intel-sponza | diffuse_1 | 0.5544 | [0.5336, 0.5759] | 0.5406 | 0.5367 | 0.4613 | yes | NO | yes |
| intel-sponza | incoherent | 0.5875 | [0.5868, 0.5882] | 0.5947 | 0.5295 | 0.4900 | yes | NO | yes |
| intel-sponza | primary | 0.5925 | [0.5842, 0.6006] | 0.6381 | 0.4858 | 0.5987 | yes | NO | inconclusive |
| intel-sponza | shadow_ao | 0.5398 | [0.5366, 0.5432] | 0.5370 | 0.5315 | 0.4619 | yes | NO | yes |
| san-miguel | diffuse_1 | 0.5861 | [0.5834, 0.5891] | 0.5891 | 0.4755 | 0.5145 | yes | NO | yes |
| san-miguel | incoherent | 0.5838 | [0.5830, 0.5847] | 0.5760 | 0.4767 | 0.4909 | yes | yes | yes |
| san-miguel | primary | 0.5965 | [0.5836, 0.6090] | 0.5936 | 0.4001 | 0.5573 | yes | NO | yes |
| san-miguel | shadow_ao | 0.5809 | [0.5769, 0.5851] | 0.5612 | 0.4780 | 0.4232 | yes | yes | yes |

- **Above chance in 12 of 12 cells.** Every CI lower bound exceeds 0.5.
- **Beats `box_projected_ratio` in 11 of 12** by a paired CI that excludes zero;
  the twelfth (intel-sponza primary) is inconclusive, not a loss. This is the
  criterion the plan called the most important, because `box_projected_ratio`
  uses no triangle information at all. The triangle descriptor adds something
  the bounding box does not contain.
- **Beats both density controls in 6 of 12.** `surface_density` is a strong
  control and is genuinely better on intel-sponza primary (0.6381 vs 0.5925).

The earlier 4-scene run agrees and adds Bistro (0.5927-0.6080 across its four
distributions, beating `box_projected_ratio` in 4 of 4).

**The effect is real but modest.** A win rate near 0.585 means the descriptor
picks the right sibling on roughly 58-59% of the pairs where it expresses a
preference. That is well above chance and well above the box-only control, but
it is not a strong classifier.

---

## 2. Gate B -- does min-fill beat the summed loss? **FAIL**

WP-B added `directional_min`, `directional_softmin` and `directional_spread`
using per-axis fills `q_i = G_i / F_i` and the saturating map
`fill(q) = 1 - exp(-q)`, with `L = SA * (1 - shape)`. The probe scores two of
these directly:

- `directional_mean_fill` = `1 - Ldir/SA` -- exactly the summed loss already
  implemented, expressed on the fill scale.
- `directional_min_fill` = the min-axis fill, degenerate axes excluded.

Paired bootstrap, `directional_min_fill` minus `directional_mean_fill`:

| scene | ray dist | min_fill | mean_fill | delta | 95% CI of delta | verdict |
|---|---|---|---|---|---|---|
| hairball | diffuse_1 | 0.5669 | 0.5838 | -0.0169 | [-0.0231, -0.0105] | worse |
| hairball | incoherent | 0.5682 | 0.5871 | -0.0189 | [-0.0196, -0.0183] | worse |
| hairball | primary | 0.5672 | 0.5947 | -0.0275 | [-0.0333, -0.0217] | worse |
| hairball | shadow_ao | 0.5710 | 0.5857 | -0.0148 | [-0.0213, -0.0088] | worse |
| intel-sponza | diffuse_1 | 0.5371 | 0.5547 | -0.0176 | [-0.0401, +0.0079] | inconclusive |
| intel-sponza | incoherent | 0.5578 | 0.6038 | -0.0460 | [-0.0469, -0.0451] | worse |
| intel-sponza | primary | 0.5105 | 0.5774 | -0.0669 | [-0.0768, -0.0568] | worse |
| intel-sponza | shadow_ao | 0.5226 | 0.5853 | -0.0628 | [-0.0663, -0.0588] | worse |
| san-miguel | diffuse_1 | 0.5368 | 0.6005 | -0.0636 | [-0.0669, -0.0600] | worse |
| san-miguel | incoherent | 0.5175 | 0.6248 | -0.1073 | [-0.1084, -0.1062] | worse |
| san-miguel | primary | 0.5295 | 0.6415 | -0.1119 | [-0.1267, -0.0978] | worse |
| san-miguel | shadow_ao | 0.5290 | 0.6261 | -0.0971 | [-0.1027, -0.0919] | worse |

**Significantly worse in 11 of 12 cells, inconclusive in 1, better in 0.** The
margin is not marginal: on San Miguel min-fill gives up 6 to 11 percentage
points of win rate, more than the entire margin by which `directional` beats
`box_projected_ratio` in the first place.

`directional_min_fill` still beats `box_projected_ratio` in 10 of 12 cells, so
it is not useless -- it is simply a worse use of the same descriptor. Taking the
minimum discards the other two axes, and those axes carry signal.

Per the plan's own ordering, Gate B gates the collapse runs. It failed, so no
collapse run was made with a min-based loss and no Gate C result is claimed.

---

## 3. d2 -- why Gate B failed

The §2 diagnosis was that summing over axes is the isotropic case and erases the
directional signal. The plan set out the test explicitly: *"Are the three fills
correlated or anticorrelated? Correlated means the geometry is isotropic and
summing is harmless. Anticorrelated means summing is destroying real signal."*

Measured over **every** binary node (33.2M nodes across the three scenes; the
CSV sample is strided, these statistics are not):

| scene | binary nodes | depth | saturation rate | corr(fill_x, fill_y) | corr(x,z) | corr(y,z) |
|---|---|---|---|---|---|---|
| intel-sponza | 7,493,895 | 31 | 0.0191 | -0.0547 | -0.1584 | -0.0124 |
| hairball | 5,759,999 | 34 | 0.0208 | +0.6529 | +0.6602 | +0.6611 |
| san-miguel | 19,943,025 | 62 | 0.0387 | +0.1465 | +0.4454 | +0.1644 |

**The fills are positively correlated or near zero. They are never
anticorrelated.** By the plan's own criterion this is the "summing is harmless"
branch. Hairball, the scene whose geometry is most obviously anisotropic at the
triangle level, has the *most strongly correlated* per-axis fills of the three
(+0.65 to +0.66) -- its hair strands are isotropically distributed in aggregate,
whatever any individual strand does.

Two further predictions from the plan are also falsified:

- **Saturation.** Predicted "low on Sponza, high on Hairball." Measured 1.91% on
  Sponza and 2.08% on Hairball -- both low, and effectively equal. San Miguel is
  highest at 3.87%. Between 96% and 98% of axes never saturate, which is why
  replacing the clamp with `1 - exp(-q)` changes little in practice. Median `q`
  is 0.185 (Hairball) to 0.485 (Sponza) and p90 stays below 0.90 on all three
  scenes, so fewer than 10% of axes per scene reach the region where a clamp
  at q=1 would bind at all.
- **Scale invariance.** The plan predicted the area-based loss would fail it.
  It does not. `SA` and every loss scale as `s^2`, so the DP objective is
  multiplied by a positive constant and the argmin is unchanged. The unit test
  `UniformScaleDoesNotChangeWhichDecisionsTheLossChanges` confirms this; the
  residual differences are the binary binned-SAH builder's f32 centroid binning,
  not the loss, which is why the test measures each loss against its own scale's
  SAH baseline.

The A/B example in `SummedLossCannotSeparateAnisotropyButMinCan` is still
correct mathematics: per-axis empty areas `[10,0,0]` and `[3.33,3.33,3.33]` both
give `Ldir = 20`, and min separates them. The defect is real. It is just rare in
these scenes, and the price of fixing it -- throwing away two of three axes -- is
much larger than the defect.

**Degenerate axes** (the guard WP-B added, policy `exclude`): 0 nodes on
Hairball, 71,302 on Sponza (0.95%), 367,536 on San Miguel (1.84%). Both policies
are unit-tested; the choice does not affect any conclusion here.

---

## 4. The unexpected result: the useful signal is not directional

`directional_mean_fill` is a **direction-free** node score -- it uses only the
node's box and its triangle descriptor, no ray. `directional` is the
**direction-conditioned** score, evaluated along each ray's actual direction.
Paired bootstrap, `directional_mean_fill` minus `directional`:

| scene | ray dist | mean_fill | directional | delta | 95% CI of delta | verdict |
|---|---|---|---|---|---|---|
| hairball | diffuse_1 | 0.5838 | 0.5829 | +0.0009 | [-0.0043, +0.0062] | inconclusive |
| hairball | incoherent | 0.5871 | 0.5835 | +0.0036 | [+0.0030, +0.0042] | better |
| hairball | primary | 0.5947 | 0.5853 | +0.0095 | [+0.0047, +0.0146] | better |
| hairball | shadow_ao | 0.5857 | 0.5859 | -0.0002 | [-0.0057, +0.0053] | inconclusive |
| intel-sponza | diffuse_1 | 0.5547 | 0.5544 | +0.0003 | [-0.0186, +0.0209] | inconclusive |
| intel-sponza | incoherent | 0.6038 | 0.5875 | +0.0164 | [+0.0157, +0.0170] | better |
| intel-sponza | primary | 0.5774 | 0.5925 | -0.0151 | [-0.0230, -0.0071] | worse |
| intel-sponza | shadow_ao | 0.5853 | 0.5398 | +0.0455 | [+0.0424, +0.0487] | better |
| san-miguel | diffuse_1 | 0.6005 | 0.5861 | +0.0144 | [+0.0115, +0.0172] | better |
| san-miguel | incoherent | 0.6248 | 0.5838 | +0.0410 | [+0.0402, +0.0419] | better |
| san-miguel | primary | 0.6415 | 0.5965 | +0.0449 | [+0.0330, +0.0587] | better |
| san-miguel | shadow_ao | 0.6261 | 0.5809 | +0.0451 | [+0.0408, +0.0495] | better |

**Better in 8 cells, inconclusive in 3, worse in 1.** Conditioning on the ray
direction does not help and usually hurts. It also beats `box_projected_ratio`
in 11 of 12 cells by +0.068 to +0.203 -- consistently larger margins than
`directional` achieves.

This reframes the whole direction. The triangle descriptor carries real,
reproducible information about which sibling is worth entering, and that
information survives -- improves -- when the ray direction is thrown away. What
the probe has been measuring as a "directional" effect is an **emptiness**
effect: how much of the node's projected area is not covered by triangle area,
summed over axes. That is precisely the quantity `Ldir` already computes, and
`directional_mean_fill = 1 - Ldir/SA` is that quantity on the fill scale.

The one consistent exception across every table is **intel-sponza primary**,
where `box_projected_ratio` (0.5987) and `surface_density` (0.6381) both beat
every descriptor score. Primary rays on Sponza are a coherent bundle hitting
large flat architecture; box geometry alone predicts well there.

---

## 5. Verdicts

| gate | criterion | result |
|---|---|---|
| V0 | mu=0 byte-identical for all five loss kinds | pass (`EveryVariantIsByteIdenticalAtMuZero`) |
| V2 | anisotropy discrimination | pass (`SummedLossCannotSeparateAnisotropyButMinCan`) |
| **A** | directional beats 0.5, the density controls, and `box_projected_ratio` | **pass** -- 12/12 above chance, 11/12 beat `box_projected_ratio` |
| **B** | `directional_min` beats the current summed loss | **fail** -- worse in 11/12, better in 0 |
| C | traversal work reduced, visit-weighted | not run; gated on B |

256 unit tests pass; build is clean (0 warnings, 0 errors).

## 6. What this means for the next step

1. **Do not adopt a min-based or spread-based loss.** The data says the summed
   loss is the better use of this descriptor, by a wide and consistent margin.
   The three new loss kinds stay in the tree as tested, documented, non-default
   variants; `collapse_loss::directional` remains the one to use.

2. **Stop calling it directional.** The signal is direction-free emptiness.
   Removing the direction conditioning made the score *better* in 8 of 12 cells.
   Any further work should be framed as a projected-emptiness term, and the
   ray-conditioned framing should be retired rather than repaired.

3. **The prediction result does not license a performance claim.** A 0.585 win
   rate on discordant sibling pairs is evidence that the term ranks siblings
   better than chance. It is not evidence that a collapse using it traverses
   less. That is Gate C, it is unrun here, and the existing collapse result at
   `6097937` is the only measured traversal evidence in this worktree.

4. **The honest open question** is whether an effect of this size at the sibling
   level is large enough to move traversal counters at all after the DP collapse
   has aggregated it over millions of nodes. Nothing measured so far answers
   that, and the sibling win rate alone cannot.
