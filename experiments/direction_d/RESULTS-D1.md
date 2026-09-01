# Direction D — D1 mechanism report (preliminary, development scenes only)

**Date:** 2026-08-31
**Run id:** `d1_dev_2026_08_31`
**Scope:** Work Packages 0–3 of `PLAN-direction-D-reviewed.md`. WP4 and later were
not started: the builder, the directional split cost, the λ grid and every
builder-performance claim are out of scope here and none of them exist in this
tree.

The experiment contract was frozen in `experiments/direction_d/README.md`
*before* any scene measurement was inspected. Nothing below was chosen after
seeing a number.

---

## 1. Provenance

| | |
|---|---|
| base commit | `3e7c361` (`Stabilize overlap metrics and experiment artifacts`) |
| WP0 + WP1 | `983bc54` |
| WP2 | `27b7732` |
| working tree that produced these artifacts | `27b7732` + the WP3 sources introduced by the commit that carries this file. Every CSV row therefore records `git_commit=27b7732`, `dirty=1`. |
| builder | `binned_sah`, 32 bins, `max_leaf_size=1`, robust mode |
| layout | `bvh2 / slot32_aos` |
| threads | 1 |
| resolution | 512×512 (see §3 for the two coordinates that walked the frozen ladder) |
| ray sets | `primary` (seed A only — primary rays are seed independent), `shadow_ao`, `diffuse_1`, `incoherent` (seeds A and B) |
| coordinates | 4 scenes × 7 = **28**, all completed, none excluded |

Build: Release x64, **0 warnings, 0 errors**. Tests: **238 passed, 0 failed**
(208 at the untouched baseline).

Commands:

```powershell
& 'C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe' `
  bvh-lab.sln /m /p:Configuration=Release /p:Platform=x64
& '.\x64\Release\bvh-test.exe'

foreach ($s in @('teapot.obj','cornell-box.obj','bunny.obj','utah_teapot.obj')) {
  & '.\x64\Release\trax.exe' --direction_d_only --run_id=d1_dev_2026_08_31 `
    "--scene=scenes/$s" --width=512 --height=512 --rays=262144 --bins=32 `
    --threads=1 --git_commit=27b7732 --dirty=1
}

python experiments/direction_d/aggregate_d1.py results/d1_dev_2026_08_31
```

Every ray set was generated **once in memory** from the canonical tree, hashed,
validated ray-by-ray against the brute-force oracle, and then reused for the
analysis. The on-disk rayset cache was never consulted. Every one of the 28
coordinates reports `validation=oracle_match`; **0 rays disagreed with the
oracle across all 5,142,524 rays**.

---

## 2. What these numbers are, and what they are not

* `Q_raw` is a **surface ratio**; `E_lower = 1 - clamp(Q_raw,0,1)` is a **lower
  bound on empty projected area**. Neither is a miss probability. Neither knows
  anything about ray origins, finite ranges, visibility or occlusion.
* A **candidate event** is one child whose AABB test passed at one parent visit.
  An **AABB false positive** is a candidate whose independent subtree query
  found no relevant intersection.
* The `probe_*` counters are **counterfactual child costs** — the work that
  *would* be performed below that child under the snapshot limit. They are not
  the work the production traversal performed, and they support no runtime
  claim. Production counters are reported separately in §7.
* `shadow_ao` is the repository's **finite ambient-occlusion** distribution
  (`t_max = 0.1 × scene diagonal`), not a direct-light shadow workload.
* The four bundled scenes are `role=development` in `scenes.csv`. They cannot
  satisfy the substantive-scene precondition of the formal D1 gate.

---

## 3. Support

Thresholds (frozen): ≥ 10,000 valid candidate events **and** ≥ 1,000 discordant
sibling pairs.

| scene | ray_set | seed | res | rays | candidates | valid cand | pairs | rays w/ pairs | invalid % | saturated % | AABB FP % | support |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| teapot.obj | primary | A | 512x512 | 262144 | 830901 | 830901 | 82518 | 26597 | 0.00 | 22.16 | 61.06 | yes |
| teapot.obj | shadow_ao | A | 512x512 | 26792 | 318270 | 318270 | 4121 | 1233 | 0.00 | 25.52 | 95.54 | yes |
| teapot.obj | shadow_ao | B | 512x512 | 26792 | 317668 | 317668 | 4176 | 1240 | 0.00 | 25.51 | 95.49 | yes |
| teapot.obj | diffuse_1 | A | 512x512 | 26792 | 336145 | 336145 | 6890 | 2020 | 0.00 | 25.09 | 92.85 | yes |
| teapot.obj | diffuse_1 | B | 512x512 | 26792 | 335706 | 335706 | 6953 | 2039 | 0.00 | 25.02 | 92.78 | yes |
| teapot.obj | incoherent | A | 512x512 | 262144 | 3241081 | 3241081 | 357233 | 115462 | 0.00 | 24.63 | 59.19 | yes |
| teapot.obj | incoherent | B | 512x512 | 262144 | 3240614 | 3240614 | 355935 | 115258 | 0.00 | 24.64 | 59.27 | yes |
| cornell-box.obj | primary | A | 512x512 | 262144 | 1076863 | 1076863 | 202571 | 85465 | 0.00 | 32.04 | 33.79 | yes |
| cornell-box.obj | shadow_ao | A | 512x512 | 85465 | 428902 | 428902 | 13049 | 4640 | 0.00 | 40.95 | 90.66 | yes |
| cornell-box.obj | shadow_ao | B | 512x512 | 85465 | 429070 | 429070 | 13061 | 4686 | 0.00 | 40.93 | 90.56 | yes |
| cornell-box.obj | diffuse_1 | A | 512x512 | 85465 | 650333 | 650333 | 98524 | 36277 | 0.00 | 32.44 | 54.82 | yes |
| cornell-box.obj | diffuse_1 | B | 512x512 | 85465 | 648698 | 648698 | 97610 | 36045 | 0.00 | 32.46 | 55.12 | yes |
| cornell-box.obj | incoherent | A | 512x512 | 262144 | 2891089 | 2891089 | 511257 | 219888 | 0.00 | 34.20 | 42.22 | yes |
| cornell-box.obj | incoherent | B | 512x512 | 262144 | 2894567 | 2894567 | 512373 | 220010 | 0.00 | 34.20 | 42.20 | yes |
| bunny.obj | primary | A | 512x512 | 262144 | 1414090 | 1414090 | 172604 | 41729 | 0.00 | 23.12 | 56.51 | yes |
| **bunny.obj** | **shadow_ao** | **A** | **2048x2048** | **669032** | **0** | **0** | **0** | **0** | – | – | – | **NO** |
| **bunny.obj** | **shadow_ao** | **B** | **2048x2048** | **669032** | **0** | **0** | **0** | **0** | – | – | – | **NO** |
| bunny.obj | diffuse_1 | A | 512x512 | 41818 | 146756 | 146756 | 9596 | 2237 | 0.00 | 26.43 | 78.88 | yes |
| bunny.obj | diffuse_1 | B | 512x512 | 41818 | 146289 | 146289 | 9604 | 2231 | 0.00 | 26.60 | 78.84 | yes |
| bunny.obj | incoherent | A | 512x512 | 262144 | 2775298 | 2775298 | 314700 | 77080 | 0.00 | 18.47 | 61.52 | yes |
| bunny.obj | incoherent | B | 512x512 | 262144 | 2774917 | 2774917 | 314686 | 77183 | 0.00 | 18.46 | 61.47 | yes |
| utah_teapot.obj | primary | A | 512x512 | 262144 | 1353092 | 1353092 | 145899 | 31464 | 0.00 | 47.93 | 57.47 | yes |
| utah_teapot.obj | shadow_ao | A | 512x512 | 31517 | 576175 | 576175 | 11164 | 2463 | 0.00 | 55.19 | 92.81 | yes |
| utah_teapot.obj | shadow_ao | B | 512x512 | 31517 | 578955 | 578955 | 11431 | 2506 | 0.00 | 55.07 | 92.70 | yes |
| utah_teapot.obj | diffuse_1 | A | 512x512 | 31517 | 614558 | 614558 | 14447 | 2984 | 0.00 | 54.03 | 91.47 | yes |
| utah_teapot.obj | diffuse_1 | B | 512x512 | 31517 | 617792 | 617792 | 14628 | 3015 | 0.00 | 53.91 | 91.43 | yes |
| utah_teapot.obj | incoherent | A | 512x512 | 262144 | 4772878 | 4772878 | 517183 | 120305 | 0.00 | 49.09 | 55.44 | yes |
| utah_teapot.obj | incoherent | B | 512x512 | 262144 | 4765384 | 4765384 | 515627 | 120101 | 0.00 | 49.14 | 55.44 | yes |

**26 of 28 rows meet the support thresholds.** Totals across all 28: 5,142,524
rays, 37,129,574 parent visits, 38,176,091 candidate events, 4,317,840
discordant pairs, 1,354,158 rays carrying at least one pair.

### The two zero-support rows, explained rather than hidden

`bunny.obj / shadow_ao` produced **zero candidate events at every rung of the
frozen resolution ladder** (512 → 1024 → 2048). The cause is arithmetic, not a
Direction D defect:

```
bunny.obj bounds diagonal        = 0.24938
AO t_max = 0.1 * diagonal        = 0.02494
ray::t_min_default = 1 / (1<<5)  = 0.03125     >  t_max
```

Every bunny AO ray therefore has an empty valid range `[t_min, t_max)`, so no
box test can pass and no occluder can be found. This is a pre-existing property
of `rayset::generate` combined with the *absolute* `t_min_default` in
`core/ray.h`, on a scene whose absolute scale is small. Validation agrees with
it: the BVH and the brute-force oracle both report "not occluded" for all
669,032 rays, so the coordinate is internally consistent and simply carries no
information. `directional_accuracy` and every derived column are written
**empty**, not `0.0`, because they are undefined with zero pairs.

The other three scenes have diagonals of 2.55, 3.28 and 8.21, so their AO range
is well clear of `t_min_default`.

### Invalid and saturated descriptors

* **`invalid_events = 0` across all 28 coordinates.** No candidate landed in the
  `invalid` bucket, so no invalid ratio could contaminate a valid bin. (The
  invalid path is nevertheless exercised by
  `DirectionalProbe.InvalidDescriptorsOnlyEnterTheInvalidBucket`, which builds a
  coplanar fixture where every projected box area is exactly zero.)
* **Saturation (`Q_raw > 1`) is large and scene dependent:** 18.5–25.5% on
  teapot and bunny, 32–41% on cornell-box, and **47.9–55.2% on utah_teapot**.
  Pooled, 13,272,832 of 38,176,091 candidate events (**34.8%**) are saturated.
  That is the coincident/overlapping-projection failure mode predicted in
  `PLAN-direction-D-reviewed.md §2.2`: on a third to a half of all events the
  descriptor carries no gradation at all, only "at least full".

---

## 4. Within-parent ranking accuracy

Frozen definition: for a discordant sibling pair, a score is *correct* when the
positive child's score is strictly greater; equal scores are *ties*; a score
undefined on either side is recorded as a tie. `accuracy = correct / pairs`,
with ties in the denominator and never counted as correct or half-correct. All
five scores use the same denominator, so the deltas are directly comparable.
Intervals are ray-clustered bootstraps, B = 2000, seed `0xD1D1D1D1D1D1D1D1`.

| scene | ray_set | seed | directional | dir 95% CI | surf_density | prim_count | box_SA | box_proj | best control | best ctl | delta pp | delta 95% CI |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| teapot.obj | primary | A | 52.99 | [52.66, 53.33] | 53.09 | 19.40 | 48.72 | 48.63 | surface_density | 53.09 | -0.10 | [-0.39, +0.19] |
| teapot.obj | shadow_ao | A | 59.18 | [57.81, 60.63] | 59.89 | 33.68 | 38.73 | 38.92 | surface_density | 59.89 | -0.70 | [-1.78, +0.33] |
| teapot.obj | shadow_ao | B | 57.97 | [56.58, 59.38] | 59.91 | 34.17 | 39.15 | 39.99 | surface_density | 59.91 | -1.94 | [-2.99, -0.90] |
| teapot.obj | diffuse_1 | A | 61.94 | [60.89, 63.02] | 61.96 | 35.86 | 36.63 | 37.90 | surface_density | 61.96 | -0.01 | [-0.88, +0.83] |
| teapot.obj | diffuse_1 | B | 60.71 | [59.61, 61.79] | 61.56 | 35.18 | 36.49 | 37.45 | surface_density | 61.56 | -0.85 | [-1.66, +0.00] |
| teapot.obj | incoherent | A | 54.62 | [54.46, 54.79] | 53.35 | 29.50 | 45.35 | 47.05 | surface_density | 53.35 | +1.27 | [+1.14, +1.39] |
| teapot.obj | incoherent | B | 54.56 | [54.40, 54.73] | 53.30 | 29.49 | 45.36 | 47.06 | surface_density | 53.30 | +1.27 | [+1.14, +1.39] |
| cornell-box.obj | primary | A | **44.48** | [44.29, 44.69] | 52.19 | 17.12 | 32.77 | 33.24 | surface_density | 52.19 | **-7.70** | [-7.90, -7.51] |
| cornell-box.obj | shadow_ao | A | 52.59 | [51.92, 53.27] | 46.82 | 28.35 | 40.58 | 41.08 | surface_density | 46.82 | +5.76 | [+5.16, +6.38] |
| cornell-box.obj | shadow_ao | B | 52.32 | [51.63, 52.97] | 47.52 | 28.53 | 40.24 | 40.75 | surface_density | 47.52 | +4.80 | [+4.14, +5.42] |
| cornell-box.obj | diffuse_1 | A | 57.25 | [57.01, 57.49] | 58.20 | 14.89 | 42.40 | 42.70 | surface_density | 58.20 | -0.95 | [-1.16, -0.74] |
| cornell-box.obj | diffuse_1 | B | 57.28 | [57.04, 57.52] | 58.61 | 14.80 | 42.36 | 42.62 | surface_density | 58.61 | -1.33 | [-1.54, -1.11] |
| cornell-box.obj | incoherent | A | 52.52 | [52.40, 52.63] | 54.86 | 14.66 | 31.86 | 32.19 | surface_density | 54.86 | -2.34 | [-2.44, -2.23] |
| cornell-box.obj | incoherent | B | 52.57 | [52.46, 52.69] | 54.84 | 14.70 | 31.77 | 32.11 | surface_density | 54.84 | -2.27 | [-2.38, -2.17] |
| bunny.obj | primary | A | 53.41 | [53.16, 53.63] | 53.74 | 37.59 | 47.40 | 47.32 | surface_density | 53.74 | -0.33 | [-0.50, -0.17] |
| bunny.obj | shadow_ao | A | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a |
| bunny.obj | shadow_ao | B | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a |
| bunny.obj | diffuse_1 | A | 56.98 | [55.93, 58.03] | 55.27 | 40.27 | 48.27 | 49.33 | surface_density | 55.27 | +1.71 | [+0.96, +2.50] |
| bunny.obj | diffuse_1 | B | 54.85 | [53.86, 55.83] | 53.38 | 39.84 | 48.69 | 49.91 | surface_density | 53.38 | +1.47 | [+0.67, +2.29] |
| bunny.obj | incoherent | A | 55.82 | [55.66, 56.00] | 54.33 | 38.57 | 47.58 | 48.19 | surface_density | 54.33 | +1.49 | [+1.35, +1.62] |
| bunny.obj | incoherent | B | 56.07 | [55.90, 56.24] | 54.54 | 38.68 | 47.59 | 48.14 | surface_density | 54.54 | +1.53 | [+1.40, +1.66] |
| utah_teapot.obj | primary | A | 52.77 | [52.50, 53.02] | 52.66 | 35.55 | 48.97 | 49.32 | surface_density | 52.66 | +0.10 | [-0.06, +0.25] |
| utah_teapot.obj | shadow_ao | A | 53.68 | [52.82, 54.63] | 49.79 | 34.55 | 47.27 | 47.20 | surface_density | 49.79 | +3.89 | [+3.11, +4.68] |
| utah_teapot.obj | shadow_ao | B | 52.95 | [52.06, 53.83] | 49.13 | 34.06 | 46.33 | 46.01 | surface_density | 49.13 | +3.82 | [+3.06, +4.57] |
| utah_teapot.obj | diffuse_1 | A | 52.57 | [51.76, 53.40] | 49.28 | 34.66 | 46.10 | 46.30 | surface_density | 49.28 | +3.29 | [+2.58, +4.00] |
| utah_teapot.obj | diffuse_1 | B | 52.41 | [51.64, 53.17] | 49.80 | 33.96 | 45.78 | 45.48 | surface_density | 49.80 | +2.60 | [+1.89, +3.33] |
| utah_teapot.obj | incoherent | A | 52.13 | [52.00, 52.26] | 50.92 | 34.21 | 45.57 | 47.11 | surface_density | 50.92 | +1.21 | [+1.10, +1.32] |
| utah_teapot.obj | incoherent | B | 52.16 | [52.03, 52.30] | 51.04 | 34.08 | 45.58 | 47.11 | surface_density | 51.04 | +1.12 | [+1.00, +1.23] |

Observations, stated as measured:

1. `surface_density` is the best cheap control in **every one of the 26
   supported rows**. The other three declared controls
   (`primitive_count`, `box_surface_ratio`, `box_projected_ratio`) score
   14.7–49.9%, i.e. at or below chance under the direction frozen in advance
   ("higher predicts the hit child").
2. Direction D beats the best control on **15 of 26** supported rows and loses on
   **11**. The largest win is `+5.76 pp` (cornell-box `shadow_ao`); the largest
   loss is `-7.70 pp` (cornell-box `primary`), where the descriptor is
   **anti-predictive at 44.48%** with a tight interval that excludes 50%.
3. The pooled result is nevertheless a small deficit (−0.19 pp, §6). Counting
   correctly-ranked pairs rather than rows, Direction D is 8,126 pairs behind
   `surface_density` out of 4,317,840. The whole deficit is cornell-box: that
   scene alone is −40,054 pairs (its three worst coordinates are `primary`
   −15,603, `incoherent` seed A −11,943 and seed B −11,653), against
   +13,912 on utah_teapot, +9,233 on bunny and +8,783 on teapot.
3. The `shadow_ao` and `diffuse_1` seed-A/seed-B replicates agree closely
   (largest directional gap 2.13 pp, on bunny `diffuse_1`), so the row-level
   numbers are not seed artefacts.

### Ties

`directional` ties are 0.00% on teapot and bunny, 0.36–1.72% on utah_teapot, and
**4.7–24.9% on cornell-box** — the axis-aligned scene produces many exactly
equal projected-area sums between siblings. `primitive_count` ties on 21.9–56.6%
of pairs everywhere. Full per-row tie counts are in `d1_summary.csv`
(`<score>_ties`) and in the aggregator output.

### A pre-registration weakness worth recording

Pooled over the 26 supported rows (4,317,840 pairs):

| score | declared direction | ties | if the direction were inverted |
|---|---|---|---|
| `directional` | **53.23%** | 7.62% | 39.15% |
| `surface_density` | **53.42%** | 6.17% | 40.41% |
| `primitive_count` | 27.64% | 39.19% | 33.17% |
| `box_surface_ratio` | 42.02% | 10.31% | 47.67% |
| `box_projected_ratio` | 42.89% | 9.04% | 48.07% |

The single frozen direction ("higher predicts the hit child") is clearly the
wrong one for the two box-ratio controls: a *smaller* child box is the better
predictor. Inverting them is not done here — that would be exactly the post-hoc
control selection the plan warns against — and it would not change the verdict:
pooled, the inverted box ratios reach only 47.67% and 48.07%, still below both
`surface_density` (53.42%) and `directional` (53.23%). On individual rows the
inversion is worth more (up to 62.5% on a teapot `diffuse_1` row), so a future
revision of the contract should declare a per-control direction rather than one
shared direction.

---

## 5. Calibration: AABB false-positive rate by `Q_raw`

Pooled over all 28 coordinates (`raw_gt_1` reported separately, as frozen):

| quintile | candidates | AABB false-positive rate |
|---|---|---|
| Q1 `[0.0,0.2)` | 435,708 | **90.22%** |
| Q2 `[0.2,0.4)` | 2,858,244 | 78.40% |
| Q3 `[0.4,0.6)` | 7,720,853 | 65.22% |
| Q4 `[0.6,0.8)` | 7,222,784 | 60.59% |
| Q5 `[0.8,1.0]` | 6,665,670 | **53.89%** |
| `raw_gt_1` (saturated) | 13,272,832 | 52.32% |
| `invalid` | 0 | – |

Per bin, the relation is monotone across all ten valid bins: 95.56, 88.87,
82.38, 76.53, 67.25, 63.62, 62.22, 58.67, 58.08, 50.82 percent.

Of the 26 coordinates with a usable quintile profile, **0 have Q5 above Q1**;
15 contain at least one adjacent-bin reversal, which the frozen contract
explicitly permits.

**This is the one clearly positive finding.** A low directional surface ratio
really does mark a child that is much more likely to be an AABB false positive —
90% versus 54% between the extreme quintiles. What it does not do is win the
*within-parent* comparison against a cheap non-directional control, which is the
test that isolates incremental information from confounding by depth, size and
primitive density.

---

## 6. Aggregates

By scene (pooled over that scene's supported rows):

| scene | rows | candidates | pairs | directional % | best control | best ctl % | delta pp | AABB FP % |
|---|---|---|---|---|---|---|---|---|
| bunny.obj | 5 | 7,257,350 | 821,190 | 55.41 | surface_density | 54.29 | +1.12 | 61.22 |
| cornell-box.obj | 7 | 9,019,522 | 1,448,445 | 52.05 | surface_density | 54.82 | -2.77 | 47.65 |
| teapot.obj | 7 | 8,620,385 | 817,826 | 54.59 | surface_density | 53.51 | +1.07 | 64.70 |
| utah_teapot.obj | 7 | 13,278,834 | 1,230,379 | 52.25 | surface_density | 51.12 | +1.13 | 62.23 |

By ray set:

| ray_set | rows | candidates | pairs | directional % | best ctl % | delta pp | AABB FP % |
|---|---|---|---|---|---|---|---|
| primary | 4 | 4,674,946 | 603,592 | 50.20 | 52.87 | **-2.67** | 52.36 |
| shadow_ao | 8 | 2,649,040 | 57,002 | 53.68 | 49.93 | **+3.75** | 92.72 |
| diffuse_1 | 8 | 3,496,277 | 258,252 | 56.84 | 57.28 | -0.44 | 77.10 |
| incoherent | 8 | 27,355,828 | 3,398,994 | 53.48 | 53.28 | +0.21 | 54.77 |

By family:

| family | rows | candidates | pairs | directional % | best ctl % | delta pp | AABB FP % |
|---|---|---|---|---|---|---|---|
| architectural (`cornell-box.obj`) | 7 | 9,019,522 | 1,448,445 | 52.05 | 54.82 | -2.77 | 47.65 |
| organic (teapot, bunny, utah_teapot) | 21 | 29,156,569 | 2,869,395 | 53.82 | 52.71 | +1.11 | 62.71 |

By substantive-versus-diagnostic status:

| status | rows | candidates | pairs | directional % | best ctl % | delta pp |
|---|---|---|---|---|---|---|
| substantive | 0 | – | – | – | – | – |
| diagnostic (development scene, support met) | 26 | 38,176,091 | 4,317,840 | 53.23 | 53.42 | -0.19 |
| diagnostic (low support) | 2 | 0 | 0 | – | – | – |

The architectural-versus-organic split is the *opposite* of the direction the
mechanism story predicts. `PLAN-direction-D-reviewed.md §2.2` argues that
axis-aligned geometry is the favourable case for this descriptor; measured, the
one architectural scene is the only scene where the descriptor loses to the
cheap control, and it is the scene where the descriptor is anti-predictive on
primary rays. With one architectural scene this is an observation, not a
finding — it is exactly why the formal gate demands a substantive scene set.

### Pooled scene-level result

Pooled over the 26 supported rows: **directional 53.23%, `surface_density`
53.42%, delta −0.19 pp.**

| scene | rows | directional % | surface_density % | delta pp |
|---|---|---|---|---|
| bunny.obj | 5 | 55.41 | 54.29 | +1.12 |
| cornell-box.obj | 7 | 52.05 | 54.82 | -2.77 |
| teapot.obj | 7 | 54.59 | 53.51 | +1.07 |
| utah_teapot.obj | 7 | 52.25 | 51.12 | +1.13 |

Scene bootstrap (seed `0xD10000D1D10000D1`, B = 2000, 4 scenes):
**mean delta +0.14 pp, 95% CI [−1.79, +1.13] pp** — the interval contains zero,
and with four scenes it is very wide and dominated by whether cornell-box is
drawn.

---

## 7. Counterfactual probe cost and production traversal cost, side by side

These are two different scales and are never mixed.

| counter | production traversal (28 coordinates) | independent probes | ratio | probes attributed to AABB false positives |
|---|---|---|---|---|
| node steps | 37,129,574 | 216,959,304 | 5.84× | 86,473,292 (39.9% of probe) |
| prim steps | 4,419,665 | 55,491,666 | 12.56× | 18,407,310 (33.2%) |
| box tests | 69,116,624 | 433,918,608 | 6.28× | 172,946,584 (39.9%) |
| triangle tests | 4,419,665 | 55,491,666 | 12.56× | 18,407,310 (33.2%) |

The probe total exceeds the production total by design: the analysis probes
every box-hit child from the parent's snapshot limit, including children the
production traversal later prunes. **The 39.9% figure is the share of
*counterfactual* child work that sits under AABB false positives. It is not the
fraction of production traversal work, and it implies nothing about runtime.**
Per-bin probe and false-positive probe counters are in
`d1_directional_bins.csv`.

Production counters per coordinate are in `d1_summary.csv`
(`trace_node_steps`, `trace_prim_steps`, `trace_box_tests`, `trace_tri_tests`,
`trace_max_stack`, `trace_hits`), and `parent_visits == trace_node_steps` holds
exactly in all 28 rows.

---

## 8. Gate D0 — descriptor and probe correctness: **PASS**

| requirement | evidence |
|---|---|
| descriptor algebra, degeneracy, cancellation, saturation, direction scaling | 17 tests in `bvh-test/directional_geometry_test.cpp` |
| analysis traversal == production traversal (hit/miss, closest `t`, tie-compatible id, aggregate counters), probes off **and** on | `DirectionalProbe.AnalysisTraversalMatchesProductionOnEveryRayDistribution` over all six ray distributions; `DirectionalProbe.ProbesDoNotChangeTheMainTraversal` |
| every independent subtree query agrees with brute force over that subtree's descendant slots | `DirectionalProbe.ChildLabelsMatchBruteForceOverDescendantSlots`, `...ProbeCountersMatchAFreshIndependentQuery` |
| sibling permutation does not change child labels once matched by descendant primitive set | `DirectionalProbe.SiblingPermutationDoesNotChangeChildLabels` |
| no invalid ratio in a valid bin | `DirectionalGeometry.NoInvalidRatioEverLandsInAValidBin`, `DirectionalProbe.InvalidDescriptorsOnlyEnterTheInvalidBucket`, `...ValidEventsNeverLandInTheInvalidBucket`; and `invalid_events = 0` in the run |
| CSV counts reconcile exactly | in-runner reconciliation gate before any row is published, plus `aggregate_d1.py` re-checking every identity in §12 of the README over 1,354,158 pair rows: **all hold** |

Full suite: **238 tests, 238 passed, 0 failed, 0 skipped**, exit 0.

Determinism: the whole matrix was executed twice from a clean results directory,
and the teapot coordinates were then re-run a third time under a separate
`run_id`. `d1_directional_bins.csv` (84 rows) and `d1_directional_pairs.csv`
(263,849 rows) are **byte-identical**; in `d1_summary.csv`, **83 of 84 columns
are byte-identical** and the only differing column is `analysis_ms`, a
wall-clock timing.

---

## 9. Gate D1

### Formal: **NOT ASSESSABLE**

The formal gate requires at least four scenes with `role=substantive` in
`experiments/direction_d/scenes.csv`, including at least one architectural and
one organic scene, each meeting the support thresholds.
`scenes.csv` currently contains **0 substantive scenes** — all four bundled
scenes are `role=development`, and none of them has a recorded source URL or
license, so none may be promoted. No external scene was downloaded or
substituted.

### Provisional numeric evaluation of the three clauses

Using the 26 supported development rows, and clearly labelled as diagnostic:

| clause | threshold | measured | verdict |
|---|---|---|---|
| 1 | `Q_raw` ranks the hit child ≥ 55% with the ray-clustered 95% CI above 50%, in ≥ 3 scenes | **1 of 4 scenes** qualifies (bunny 55.41%; teapot 54.59%, utah_teapot 52.25%, cornell-box 52.05%) | **fail** |
| 2 | pooled scene-level improvement ≥ +3 pp over the best cheap control, scene-bootstrap 95% CI above zero | **−0.19 pp** pooled; scene bootstrap mean **+0.14 pp**, CI **[−1.79, +1.13] pp** (contains zero) | **fail** |
| 3 | false-positive rate does not increase from the lowest to the highest `Q` quintile, saturation reported separately | strictly decreasing 90.22 → 78.40 → 65.22 → 60.59 → 53.89%; 0 of 26 coordinates reverse end-to-end | **pass** |

**Provisional Gate D1 result: FAIL** (clauses 1 and 2 fail; clause 3 passes).
This is a provisional, diagnostic-only verdict on development scenes. It is
explicitly *not* a D1 pass, and equally it is not a formal D1 failure — the
formal gate cannot be run at all until a substantive scene set exists.

### What the result actually says

The descriptor **is** calibrated: children with a low directional surface ratio
are much more often AABB false positives (90% versus 54% across the extreme
quintiles), and that relation is monotone over all ten valid bins and never
reverses end-to-end on any coordinate.

The descriptor **does not** add usable information over a cheap non-directional
control in the within-parent test that controls for parent, ray, direction,
depth and incumbent range. Pooled, it is 0.19 pp *behind*
`triangle_surface_area_sum / SA(child_box)`, a quantity that needs no direction
at all. On the one architectural scene it is 2.77 pp behind overall and 7.70 pp
behind on primary rays, where it is anti-predictive.

The most likely mechanical reason is visible in the same tables: **34.8% of all
candidate events are saturated** (`Q_raw > 1`, rising to 55% on utah_teapot),
where the descriptor is pinned at "at least full" and carries no gradation; and
`U_axis` sums overlapping projections, so it tracks total surface per unit
projected box area — which is very nearly what the `surface_density` control
already measures, without the direction.

---

## 10. Limitations

Stated in the frozen contract before the run, and all still apply:

* **direction only** — ray origins are ignored entirely;
* `shadow_ao` is **finite AO**, not direct-light shadows;
* probe costs are **counterfactual**, not production work, and support no
  runtime or builder claim;
* all evidence here is analytic / CPU-counter evidence — **no timing claim, no
  builder claim, no split-cost claim**;
* the four bundled scenes are development scenes and do not establish
  generality; only one of them is architectural;
* `best_control` is selected by point accuracy on the same rows the delta is
  measured on, which favours the controls and is therefore conservative for
  Direction D;
* the scene bootstrap has 4 scenes, so its interval is very wide;
* `primary` exists for one camera view only; train/test view separation is a
  later work package.

## 11. Deviations from the frozen plan

1. `d1_directional_pairs.csv` is committed **gzip-compressed**
   (`d1_directional_pairs.csv.gz`, 8.67 MB) because the uncompressed artifact is
   168.7 MB. Both SHA-256 hashes are recorded in §12 and `aggregate_d1.py` reads
   either form. Nothing else about the file changed.
2. During WP2, one sentence of the frozen README's invalid-score rule was
   corrected to name all four scores that can be undefined and the exact area
   each divides by. This was done **before any scene measurement was taken** and
   changes no threshold, definition or direction.
3. `experiments/direction_d/aggregate_d1.py` was added; the README already named
   it as the aggregation command.
4. `results/.gitattributes` marks the committed artifacts `-text` so git stores
   them byte-exactly and the SHA-256 values in §12 survive any checkout. Without
   it git normalises the CRLF the C++ writer emits and the recorded hashes would
   not reproduce.

No coordinate was excluded, no threshold was moved, and no low-support row was
concealed.

## 12. Artifacts

Under `results/d1_dev_2026_08_31/`:

| file | bytes | SHA-256 |
|---|---|---|
| `d1_summary.csv` | 21,373 | `714e02c9c8b4a6ef25838f5558ea61870a8588c4f4caee9525ba8193e0ba80fc` |
| `d1_directional_bins.csv` | 102,327 | `1dcf2c21c30e7020b68e8771febd2564639c3faaae0882ab56025afaa32586f1` |
| `d1_directional_pairs.csv` (uncompressed) | 168,670,794 | `c377d6cafbb4b439d35dadc8363aac6b1cf771886318a7567a05794aeb67e4e7` |
| `d1_directional_pairs.csv.gz` (committed) | 8,670,172 | `982160685e35e736b3445b702bd6321074e4ae2ac1145a5fa584a5719171b052` |

Determinism re-run, under `results/d1_rerun_check/` (teapot only, separate
`run_id`):

| file | bytes | SHA-256 |
|---|---|---|
| `d1_summary.csv` | 6,377 | `1e33da3b48d6c860f42f5fbe407906e62617a4dd4fd76fc132d055b15a2683a6` |
| `d1_directional_bins.csv` | 25,635 | `85415a43d02cb28537ca5009293da7292c75f9a99c3efc872f0d1969ec680225` |
| `d1_directional_pairs.csv` | 31,331,735 | `42369b52e088821998e15820f14bc7c44a406c4ff3bfdd515c02e47a84de3e66` |

Scene hashes recorded in every row and cross-checked against
`experiments/direction_d/scenes.csv`:

| scene | SHA-256 |
|---|---|
| `teapot.obj` | `2135dfb46c005448ad3fcd2ca75a31204df91248bbbcb18965838af40df1c234` |
| `cornell-box.obj` | `a4107f73438e3d55c6cbed68fe0813d26dd0833e617c387bde08b0d83fbab350` |
| `bunny.obj` | `ed49d20e4802fe73bcfa8eca0a4e5fb567e703d542e482173bf3dff95cfb31f8` |
| `utah_teapot.obj` | `9bfab52dd61bbe7635b715da331e43af09325d4165d53dc2985b2cb24f554147` |

## 13. Stop

This is the Gate D1 review point. **Work Package 4 and later were not started.**
The builder, `bvh2_node`, the traversal kernels and `trace_stats` are
byte-identical to the base commit `3e7c361`; no directional split cost exists in
this tree, no λ was tuned, and no builder-performance result is claimed.

Recommended reading of this result before any further work: the mechanism's
*calibration* survives, its *incremental within-parent information over a cheap
control* does not. A substantive scene set would be needed to run the formal
gate, but on the evidence here the expected outcome of that gate is a fail, and
saturation at 35–55% of events is the first thing that would have to be fixed
for the descriptor to have room to discriminate at all.
