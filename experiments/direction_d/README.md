# Direction D — Directional Emptiness: frozen experiment contract

Status: **frozen before any scene measurement was inspected** (Work Package 0 of
`PLAN-direction-D-reviewed.md`).

Everything in this file is a pre-registration. Nothing below may be changed to
fit an observed result. If a choice has to change, the change is recorded as a
dated deviation in `RESULTS-D1.md` and the affected rows are re-measured.

---

## 1. Question

> When a ray intersects a child AABB, does the three-component projected-geometry
> descriptor distinguish children whose subtrees contain a relevant triangle
> intersection from AABB false positives?

This is a *mechanism* question about an existing binary BVH. Work Packages 0-3
do not change traversal, node layout, collapse, or split decisions.

---

## 2. Equations and names

For triangle vertices `v0`, `v1`, `v2`:

```text
c          = cross(v1 - v0, v2 - v0)
g_triangle = 0.5 * (abs(c.x), abs(c.y), abs(c.z))
G_node     = sum over the node's descendant triangles of g_triangle
```

`G.x` is the area of the triangle projected onto the YZ plane, `G.y` onto XZ,
`G.z` onto XY. In code the fields are `p_yz`, `p_xz`, `p_xy`.

For a finite nonzero ray direction `d`, with `u = d / length(d)`:

```text
P_exact(node,u) = sum over triangles of 0.5 * abs(dot(c, u))

U_axis(G,u)     = abs(u.x)*G.x + abs(u.y)*G.y + abs(u.z)*G.z

B_box(b,u)      = abs(u.x)*ey*ez + abs(u.y)*ex*ez + abs(u.z)*ex*ey
                  where (ex,ey,ez) = b.max - b.min

Q_raw           = U_axis / B_box
Q_clamped       = clamp(Q_raw, 0, 1)
E_lower         = 1 - Q_clamped
```

`U_axis >= P_exact` by the triangle inequality, and the projected *union* of the
triangles is no larger than `P_exact`. `E_lower` is therefore a lower bound on
the empty projected area inside the projected AABB.

**`E_lower` is not a miss probability.** It carries no ray-origin, finite-range,
visibility, overlap, or occlusion information.

### Machine-readable identifiers

| Quantity | Identifier |
|---|---|
| `G` | `axis_projected_area_sum` (`p_yz`, `p_xz`, `p_xy`) |
| sampled `P_exact` | `exact_projected_area_sum` |
| `U_axis` | `axis_projected_area_upper` |
| `Q_raw`, `Q_clamped` | `directional_surface_ratio_raw`, `directional_surface_ratio_clamped` |
| `E_lower` | `directional_emptiness_lower` |

The words `fill`, `coverage_probability`, `hit_probability` and "the box lied"
are not used in any machine-readable output. In prose, a child whose AABB test
passes but whose independent subtree query finds no relevant intersection is an
**AABB false positive**.

### Invalid results

If `d` is zero or non-finite, or `B_box <= 0` or non-finite, the descriptor
returns `valid = false` and `raw`/`clamped`/`emptiness_lower` are NaN. Invalid
results are never mapped to 0 or 1 and never enter a valid ratio bin.

### Precision

The node-indexed side array and all descriptor arithmetic use `double`.
Triangle vertices and ray directions remain `f32`. The descriptor lives in a
side vector indexed by node id. **`bvh2_node` is not modified.**

---

## 3. Query labels

At each parent visit the analysis traversal snapshots the active range limit and
then queries every box-hit child subtree independently, using that snapshot.

| `query_kind` | Ray sets | Positive child outcome |
|---|---|---|
| `closest_improves_incumbent` | `primary`, `reflection`, `diffuse_1`, `diffuse_n`, `incoherent` | the child subtree contains a triangle intersection with `t` in `[r.t_min, snapshot_limit)` — i.e. strictly closer than the incumbent at that parent visit |
| `any_occluder_in_range` | `shadow_ao` | the child subtree contains any triangle intersection with `t` in `[r.t_min, r.t_max)` |

`shadow_ao` is the current **finite ambient-occlusion** distribution with
`t_max = 0.1 * scene_diagonal`. It is **not** a direct-light shadow workload and
is never described as one.

A box-hit child whose independent subtree query is negative is an
**AABB false positive**.

The probes are counterfactual: they measure the work that *would* be done below
each candidate child under the snapshot limit. They never mutate the analysis
traversal's `hit`, stack, or counters. Their counters are always labelled
`probe_*` and are never described as the fraction of production traversal work.
Ordinary production trace counters are emitted separately in `d1_summary.csv`.

---

## 4. Ratio bins

Fixed, in this order:

```text
q_00_10  Q_raw in [0.0, 0.1)
q_10_20  Q_raw in [0.1, 0.2)
q_20_30  Q_raw in [0.2, 0.3)
q_30_40  Q_raw in [0.3, 0.4)
q_40_50  Q_raw in [0.4, 0.5)
q_50_60  Q_raw in [0.5, 0.6)
q_60_70  Q_raw in [0.6, 0.7)
q_70_80  Q_raw in [0.7, 0.8)
q_80_90  Q_raw in [0.8, 0.9)
q_90_100 Q_raw in [0.9, 1.0]
raw_gt_1 Q_raw > 1.0                 (saturation; reported separately)
invalid  descriptor valid == false   (never counted as a valid bin)
```

Quintiles used by Gate D1 clause 3 are formed by pairing adjacent valid bins:
`q1 = q_00_10 + q_10_20`, `q2 = q_20_30 + q_30_40`, `q3 = q_40_50 + q_50_60`,
`q4 = q_60_70 + q_70_80`, `q5 = q_80_90 + q_90_100`. `raw_gt_1` and `invalid`
are excluded from the quintile trend and reported on their own.

---

## 5. Scores compared

All five scores are evaluated on exactly the same child-candidate events.

| Score | Definition (per candidate child) |
|---|---|
| `directional` | `Q_raw` = `axis_projected_area_upper(G_child, u) / B_box(child_box, u)` |
| `surface_density` | `triangle_surface_area_sum(child) / surface_area(child_box)` |
| `primitive_count` | descendant primitive count of the child |
| `box_surface_ratio` | `surface_area(child_box) / surface_area(parent_box)` |
| `box_projected_ratio` | `B_box(child_box, u) / B_box(parent_box, u)` |

`surface_density`, `primitive_count`, `box_surface_ratio` and
`box_projected_ratio` are the **declared cheap controls**. No other control may
be introduced after results are seen.

**Declared direction, frozen in advance, identical for every score:** a *higher*
score is the prediction that this child contains the relevant intersection.

Note that `box_surface_ratio` and `box_projected_ratio` divide by a quantity
that is constant within a parent, so within a parent they rank identically to
the child's own surface area / projected box area.

---

## 6. Within-parent pair test and ranking accuracy

The primary statistic is a within-parent sibling comparison. It controls for
parent, ray, direction, depth and incumbent range.

* A **candidate event** is one child whose AABB test passed at one parent visit.
* A **discordant pair** is an unordered pair of candidate children of the *same*
  parent visit whose binary outcomes differ (exactly one is positive).

For each discordant pair and each score `s`, with `s_hit` the score of the
positive child and `s_miss` the score of the negative child:

```text
s_hit  >  s_miss   -> correct
s_hit ==  s_miss   -> tie
s_hit  <  s_miss   -> incorrect
```

**Frozen accuracy definition:**

```text
accuracy(s) = correct(s) / discordant_pairs
```

Ties are in the denominator and are **never** counted as correct and never as
half-correct. Tie counts are reported separately for every score.

**Frozen invalid rule:** if a score is undefined for either child of a pair,
that pair is recorded as a **tie** for that score — that is, not correct, still
in the denominator. A score is undefined when the area it divides by is not
positive and finite: `directional` when `B_box(child)` is not positive,
`box_projected_ratio` when `B_box(parent)` is not positive, `surface_density`
when `SA(child_box)` is not positive, and `box_surface_ratio` when
`SA(parent_box)` is not positive. `primitive_count` is always defined. Invalid
directional candidate events are additionally counted in the `invalid` ratio
bin.

The denominator `discordant_pairs` is therefore identical for all five scores,
so accuracy differences are directly comparable.

---

## 7. Inference

**The ray is the resampling cluster.** Child events and parent visits from one
ray are correlated repeated observations and are never treated as independent
samples.

Per scene x ray set:

* ray-clustered nonparametric bootstrap, `B = 2000` resamples;
* each resample draws `rays` ray-clusters with replacement from all rays of that
  ray set (rays with zero discordant pairs are real clusters that contribute
  `(0 correct, 0 pairs)`);
* statistic = `sum(correct) / sum(pairs)` over the resample; a resample with
  zero total pairs is discarded and does not contribute a percentile;
* 95% interval = the 2.5th and 97.5th percentiles of the resampled statistic;
* the **delta** interval uses the *same* resamples, statistic
  `accuracy(directional) - accuracy(best_control)`, so it is paired.

**Frozen bootstrap seed:** `0xD1D1D1D1D1D1D1D1` (PCG32 `rng(seed, stream)` with
`stream = 1` for the accuracy/delta bootstrap of every row). The seed does not
vary by scene or ray set, and the resample index sequence is therefore identical
across rows of equal ray count — that is intended and deterministic.

`best_control` is the cheap control with the highest point accuracy **on the
same rows**. This is disclosed as a selection-on-the-same-data caveat wherever
the delta is reported; it favours the controls, so it is conservative for
Direction D.

**Frozen pooled-scene bootstrap seed:** `0xD10000D1D10000D1`, `B = 2000`,
Python `random.Random(seed)`, resampling *scenes* with replacement, used only
for the pooled scene-level interval in Gate D1 clause 2.

---

## 8. Support thresholds

A scene x ray set row is **substantive** only if it has

* at least `10000` valid child-candidate events, and
* at least `1000` discordant sibling pairs.

Otherwise the row is **diagnostic-only** and may not be used for a gate
decision. Low-support rows are always published, never hidden.

If a row misses the threshold, the resolution is increased deterministically
along the frozen ladder `512 -> 1024 -> 2048` (square), and the final resolution
is recorded in the row. `incoherent` increases along `262144 -> 1048576`.

---

## 9. Development matrix

```text
scenes:        teapot.obj, cornell-box.obj, bunny.obj, utah_teapot.obj
builder:       binned_sah (Wald 2007 binned), robust mode
bins:          32
max_leaf_size: 1
layout:        bvh2 / slot32_aos
ray sets:      primary, shadow_ao, diffuse_1, incoherent
resolution:    512 x 512 initially (see the ladder in section 8)
seeds:         two frozen nonzero seeds for seeded distributions:
                 seed A = 11400714819323198485   (0x9E3779B97F4A7C15)
                 seed B = 14029467366897019727   (0xC2B2AE3D27D4EB4F)
threads:       1
```

`primary` rays do not depend on `rayset_args.seed`, so `primary` is generated
and measured **once** per scene, under seed A only. Duplicating it under seed B
would be a fake replicate. Train/test separation for any later builder work uses
different frozen camera directions through `camera::frame_bounds`, not fake
"primary seeds".

Every ray set is generated **once in memory** from the canonical `binned_sah`
tree, hashed, validated against the brute-force oracle, and then reused for the
Direction D analysis. The on-disk rayset cache is **not** used: its file-name
key omits scene content, camera, scale, AO radius, generator version and
numerical mode, so it cannot be trusted to identify a ray set.

---

## 10. Gates

### Gate D0 — descriptor and probe correctness

All must pass:

1. descriptor algebra, degeneracy, cancellation, saturation and direction-scale
   tests pass;
2. the analysis traversal returns identical hit/miss, closest `t`,
   tie-compatible primitive id, and identical aggregate counters as the
   production traversal, with probes disabled *and* with probes enabled;
3. every independent subtree query agrees with a brute-force query over that
   subtree's descendant primitive slots;
4. sibling permutation does not change independent child labels after children
   are matched by descendant primitive set;
5. no invalid ratio is placed into a valid bin;
6. CSV counts reconcile exactly with rays, parent visits, candidate events and
   pair totals.

A D0 failure is an implementation error, never a negative research result.

### Gate D1 — mechanism and incremental information

The **formal** gate runs only once at least four *substantive scenes* exist in
`scenes.csv` with `role=substantive`, including at least one `architectural` and
one `organic` family scene, each meeting section 8's support thresholds.

Pass requires all of:

1. in discordant sibling pairs, higher `Q_raw` predicts the relevant-hit child
   at least **55%** of the time, with the ray-clustered 95% interval above 50%,
   in at least three substantive scenes;
2. the pooled scene-level result improves by at least **3 percentage points**
   over the best declared cheap control, with the scene-bootstrap 95% interval
   above zero;
3. the calibration table is directionally consistent: the AABB false-positive
   rate does not increase from the lowest to the highest `Q` quintile, after
   reporting saturation separately. Isolated adjacent-bin reversals are
   acceptable; a flat or reversed overall relation is not.

The four bundled local scenes are `role=development`. They can produce real
preliminary measurements but they do **not** satisfy the substantive-scene
precondition of the formal gate. With only development scenes, Gate D1 is
reported as **not assessable (formal)** plus a **provisional** numeric
evaluation of clauses 1-3.

### Gate D2 / D3 / D4

Out of scope for Work Packages 0-3. Their thresholds live in
`PLAN-direction-D-reviewed.md` section 4 and are not restated here so that there
is exactly one authority.

---

## 11. Commands

Build:

```powershell
& 'C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe' `
  bvh-lab.sln /m /p:Configuration=Release /p:Platform=x64
```

Tests:

```powershell
& '.\x64\Release\bvh-test.exe'
```

One Direction D coordinate (repeat per scene; the same `--run_id` appends into
one results directory):

```powershell
& '.\x64\Release\trax.exe' --direction_d_only --run_id=<run_id> `
  --scene=scenes/<scene>.obj --width=512 --height=512 --bins=32 --threads=1 `
  --git_commit=<sha> --dirty=<0|1>
```

`--direction_d` adds the Direction D pass to an ordinary M1 run.
`--direction_d_only` runs Direction D and nothing else. Direction D never runs
in an ordinary M1/M3 invocation.

Aggregation (deterministic; the only RNG is the frozen pooled-scene bootstrap):

```powershell
python experiments/direction_d/aggregate_d1.py results/<run_id>
```

---

## 12. Output files

Beneath `results/<run_id>/`:

| File | Grain |
|---|---|
| `d1_directional_bins.csv` | scene x ray set x `query_kind` x `ratio_bin` |
| `d1_directional_pairs.csv` | one row per ray **that has at least one discordant sibling pair** |
| `d1_summary.csv` | scene x ray set x `query_kind` |

Rays with zero discordant pairs are deliberately not written to
`d1_directional_pairs.csv` (that would be one row per pixel per scene per ray
set). They are fully accounted for: `d1_summary.csv` carries `rays` and
`rays_with_pairs`, and the bootstrap reconstructs the zero-pair clusters from
`rays - rays_with_pairs`. Reconciliation identities that must hold exactly:

```text
sum over pair rows of discordant_pairs        == summary.discordant_pairs
sum over pair rows of <score>_correct         == summary.<score>_correct
sum over pair rows of <score>_ties            == summary.<score>_ties
sum over pair rows of candidate_events        == summary.candidate_events_in_pair_rows
count of pair rows                            == summary.rays_with_pairs
sum over bin rows of candidate_count          == summary.candidate_events
sum over bin rows of relevant_hit_count       == summary.relevant_hit_events
sum over bin rows of false_positive_count     == summary.false_positive_events
bin row `invalid`.candidate_count             == summary.invalid_events
bin row `raw_gt_1`.candidate_count            == summary.saturated_events
relevant_hit_count + false_positive_count     == candidate_count   (per bin row)
```

The report is `experiments/direction_d/RESULTS-D1.md`.

---

## 13. Declared limitations

These are stated up front, not discovered afterwards:

* the descriptor uses **direction only**; ray origins are ignored entirely;
* `shadow_ao` is finite AO, not direct-light shadows;
* the probe costs are **counterfactual** child costs, not production traversal
  work, and imply nothing about runtime;
* all evidence in Work Packages 0-3 is analytic / CPU-counter evidence — no
  timing claim, no builder claim;
* the bundled scenes are development scenes and do not establish generality;
* `best_control` is selected on the same rows the delta is measured on.
