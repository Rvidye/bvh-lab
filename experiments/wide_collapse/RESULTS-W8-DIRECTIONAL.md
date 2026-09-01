# Width-8 directional wide collapse — measured result

**Date:** 2026-09-01 · **Run id:** `w8_dir_2026_09_01` · **Base commit:** `af0cb56`

**Question.** Does adding workload-directional projected-area information to the
existing SAH wide-collapse cost produce a width-8 tree that performs less
traversal work than the ordinary SAH-collapsed width-8 tree?

**Answer: mixed.** It clearly helps on two scenes, does nothing useful on one,
and mildly hurts on one. The direction of the effect is not predictable in
advance, and the largest single win is on a *held-out* view rather than the
trained one.

---

## Setup

One binary SAH tree per scene (`binned_sah`, 32 bins, `max_leaf_size=1`), shared
by every variant. Width 8 only. Baseline = the existing DP collapse using surface
area. Directional variants replace that surface area with

```
Aeff(n) = (1 - lambda) * SA(B_n) + lambda * 4 * Adir(B_n; W)
Adir(B; W) = Wx*Fx + Wy*Fy + Wz*Fz        W = (E|d.x|, E|d.y|, E|d.z|)
```

at λ = 0.25, 0.5, 0.75, 1.0. `W` is computed from **view A primary rays only**.
Views B and C are held out. 512×512 primary rays per view, generated once per
view and reused byte-for-byte by every tree.

`W` measured per scene (view A):

| scene | Wx | Wy | Wz |
|---|---|---|---|
| intel-sponza | 0.87808 | 0.38754 | 0.20696 |
| bistro | 0.94388 | 0.12398 | 0.26319 |
| hairball | 0.50001 | 0.71272 | 0.45324 |
| san-miguel | 0.93691 | 0.14672 | 0.26748 |

Isotropic would be (0.5, 0.5, 0.5); three of the four camera workloads are
strongly anisotropic, hairball's least so.

**Correctness: 60 of 60 rows pass.** Every directional tree returns identical
hit/miss and identical closest-hit distance to the binary tree on **every one of
the 262,144 rays per view** — 15.7 M ray comparisons in total, 0 mismatches and
0 tie-broken ids. In addition, λ=0 through the directional weight path reproduces
the baseline collapse **exactly** (node count, depth and the full
kept/absorbed decision vector) on all four scenes.

---

## Results

`Δ` columns are percentage change versus the baseline width-8 tree on the *same*
rays. Negative is better.

| scene | view | λ | nodes | depth | dec. chg % | node visits/ray | Δ | box tests/ray | Δ | prim tests/ray | Δ |
|---|---|---|---|---|---|---|---|---|---|---|---|
| intel-sponza | **A** | base | 4,484,656 | 14 | – | 8.754 | – | 61.08 | – | 2.088 | – |
| intel-sponza | **A** | 0.25 | 4,484,961 | 14 | 2.69 | 8.754 | +0.00 | 61.08 | +0.00 | 2.088 | +0.00 |
| intel-sponza | **A** | 0.50 | 4,485,991 | 14 | 4.41 | 7.807 | **−10.81** | 53.51 | **−12.40** | 2.089 | +0.05 |
| intel-sponza | **A** | 0.75 | 4,487,766 | 14 | 5.59 | 7.794 | **−10.96** | 53.38 | **−12.60** | 2.089 | +0.05 |
| intel-sponza | **A** | 1.00 | 4,490,938 | 15 | 6.55 | 7.763 | **−11.32** | 53.25 | **−12.83** | 2.089 | +0.05 |
| intel-sponza | B | base | 4,484,656 | 14 | – | 13.213 | – | 96.26 | – | 2.992 | – |
| intel-sponza | B | 0.25 | 4,484,961 | 14 | 2.69 | 13.198 | −0.12 | 96.13 | −0.13 | 2.992 | −0.00 |
| intel-sponza | B | 0.50 | 4,485,991 | 14 | 4.41 | 12.780 | **−3.28** | 92.91 | −3.48 | 2.992 | −0.00 |
| intel-sponza | B | 0.75 | 4,487,766 | 14 | 5.59 | 12.781 | −3.27 | 92.92 | −3.47 | 2.992 | −0.01 |
| intel-sponza | B | 1.00 | 4,490,938 | 15 | 6.55 | 12.867 | −2.62 | 93.60 | −2.75 | 2.992 | −0.01 |
| intel-sponza | C | base | 4,484,656 | 14 | – | 18.578 | – | 140.30 | – | 2.890 | – |
| intel-sponza | C | 0.25 | 4,484,961 | 14 | 2.69 | 18.449 | −0.69 | 139.27 | −0.74 | 3.117 | +7.83 |
| intel-sponza | C | 0.50 | 4,485,991 | 14 | 4.41 | 17.527 | **−5.66** | 131.83 | −6.03 | 3.119 | +7.91 |
| intel-sponza | C | 0.75 | 4,487,766 | 14 | 5.59 | 17.555 | −5.51 | 131.60 | −6.20 | 2.889 | −0.04 |
| intel-sponza | C | 1.00 | 4,490,938 | 15 | 6.55 | 18.484 | −0.51 | 139.27 | −0.73 | 2.892 | +0.05 |
| bistro | **A** | base | 3,401,720 | 17 | – | 24.524 | – | 185.71 | – | 10.854 | – |
| bistro | **A** | 0.25 | 3,403,067 | 17 | 2.38 | 24.500 | −0.10 | 185.52 | −0.10 | 10.850 | −0.03 |
| bistro | **A** | 0.50 | 3,405,295 | 16 | 4.10 | 25.169 | +2.63 | 190.92 | +2.80 | 10.802 | −0.47 |
| bistro | **A** | 0.75 | 3,410,284 | 17 | 5.52 | 25.223 | +2.85 | 191.41 | +3.07 | 10.987 | +1.22 |
| bistro | **A** | 1.00 | 3,416,997 | 17 | 6.75 | 25.496 | +3.96 | 193.44 | +4.16 | 11.053 | +1.84 |
| bistro | B | base | 3,401,720 | 17 | – | 29.139 | – | 216.45 | – | 51.960 | – |
| bistro | B | 0.25 | 3,403,067 | 17 | 2.38 | 29.125 | −0.05 | 216.30 | −0.07 | 51.958 | −0.01 |
| bistro | B | 0.50 | 3,405,295 | 16 | 4.10 | 28.722 | −1.43 | 215.11 | −0.62 | 51.950 | −0.02 |
| bistro | B | 0.75 | 3,410,284 | 17 | 5.52 | 29.646 | +1.74 | 222.37 | +2.74 | 52.058 | +0.19 |
| bistro | B | 1.00 | 3,416,997 | 17 | 6.75 | 31.215 | +7.12 | 231.63 | +7.01 | 52.172 | +0.41 |
| bistro | C | base | 3,401,720 | 17 | – | 35.176 | – | 269.45 | – | 15.874 | – |
| bistro | C | 0.25 | 3,403,067 | 17 | 2.38 | 34.873 | −0.86 | 267.07 | −0.88 | 15.851 | −0.14 |
| bistro | C | 0.50 | 3,405,295 | 16 | 4.10 | 33.883 | −3.67 | 259.24 | −3.79 | 15.865 | −0.06 |
| bistro | C | 0.75 | 3,410,284 | 17 | 5.52 | 35.228 | +0.15 | 269.87 | +0.16 | 16.032 | +0.99 |
| bistro | C | 1.00 | 3,416,997 | 17 | 6.75 | 34.943 | −0.66 | 266.85 | −0.96 | 15.922 | +0.30 |
| hairball | **A** | base | 3,409,544 | 16 | – | 28.348 | – | 209.36 | – | 29.372 | – |
| hairball | **A** | 0.25 | 3,409,466 | 16 | 0.80 | 28.192 | −0.55 | 208.13 | −0.59 | 29.371 | −0.01 |
| hairball | **A** | 0.50 | 3,409,510 | 16 | 1.51 | 28.117 | −0.82 | 207.48 | −0.90 | 29.375 | +0.01 |
| hairball | **A** | 0.75 | 3,409,442 | 16 | 2.08 | 28.149 | −0.70 | 207.80 | −0.75 | 29.375 | +0.01 |
| hairball | **A** | 1.00 | 3,409,475 | 16 | 2.55 | 28.248 | −0.35 | 208.63 | −0.35 | 29.394 | +0.07 |
| hairball | B | base | 3,409,544 | 16 | – | 32.783 | – | 245.63 | – | 28.311 | – |
| hairball | B | 0.25 | 3,409,466 | 16 | 0.80 | 33.338 | +1.69 | 250.08 | +1.81 | 28.312 | +0.00 |
| hairball | B | 0.50 | 3,409,510 | 16 | 1.51 | 31.544 | −3.78 | 235.72 | −4.03 | 28.311 | −0.00 |
| hairball | B | 0.75 | 3,409,442 | 16 | 2.08 | 31.632 | −3.51 | 236.56 | −3.69 | 28.316 | +0.02 |
| hairball | B | 1.00 | 3,409,475 | 16 | 2.55 | 31.642 | −3.48 | 236.48 | −3.72 | 28.316 | +0.02 |
| hairball | C | base | 3,409,544 | 16 | – | 28.088 | – | 208.91 | – | 24.135 | – |
| hairball | C | 0.25 | 3,409,466 | 16 | 0.80 | 28.610 | +1.86 | 212.53 | +1.73 | 23.945 | −0.78 |
| hairball | C | 0.50 | 3,409,510 | 16 | 1.51 | 29.488 | **+4.99** | 219.08 | +4.87 | 24.323 | +0.78 |
| hairball | C | 0.75 | 3,409,442 | 16 | 2.08 | 29.471 | +4.92 | 218.98 | +4.82 | 24.331 | +0.81 |
| hairball | C | 1.00 | 3,409,475 | 16 | 2.55 | 29.482 | +4.96 | 219.05 | +4.86 | 24.250 | +0.48 |
| san-miguel | **A** | base | 11,927,443 | 49 | – | 6.509 | – | 43.22 | – | 1.911 | – |
| san-miguel | **A** | 0.25 | 11,930,689 | 50 | 2.57 | 6.608 | +1.52 | 41.65 | −3.64 | 1.890 | −1.13 |
| san-miguel | **A** | 0.50 | 11,937,711 | 50 | 4.26 | 6.599 | +1.39 | 42.37 | −1.97 | 1.886 | −1.30 |
| san-miguel | **A** | 0.75 | 11,950,186 | 51 | 5.55 | 6.588 | +1.21 | 42.35 | −2.01 | 1.886 | −1.30 |
| san-miguel | **A** | 1.00 | 11,969,091 | 51 | 6.65 | 6.595 | +1.32 | 42.41 | −1.88 | 1.886 | −1.30 |
| san-miguel | B | base | 11,927,443 | 49 | – | 18.695 | – | 140.59 | – | 3.874 | – |
| san-miguel | B | 0.25 | 11,930,689 | 50 | 2.57 | 15.809 | **−15.44** | 117.39 | **−16.50** | 3.849 | −0.66 |
| san-miguel | B | 0.50 | 11,937,711 | 50 | 4.26 | 15.749 | **−15.76** | 116.88 | **−16.86** | 3.845 | −0.75 |
| san-miguel | B | 0.75 | 11,950,186 | 51 | 5.55 | 16.006 | −14.38 | 118.22 | −15.91 | 3.906 | +0.82 |
| san-miguel | B | 1.00 | 11,969,091 | 51 | 6.65 | 17.440 | −6.72 | 129.23 | −8.08 | 3.917 | +1.10 |
| san-miguel | C | base | 11,927,443 | 49 | – | 27.678 | – | 211.94 | – | 4.825 | – |
| san-miguel | C | 0.25 | 11,930,689 | 50 | 2.57 | 28.155 | +1.72 | 215.73 | +1.79 | 4.831 | +0.11 |
| san-miguel | C | 0.50 | 11,937,711 | 50 | 4.26 | 28.079 | +1.45 | 215.08 | +1.48 | 4.829 | +0.07 |
| san-miguel | C | 0.75 | 11,950,186 | 51 | 5.55 | 28.192 | +1.86 | 215.98 | +1.91 | 4.829 | +0.07 |
| san-miguel | C | 1.00 | 11,950,186 | 51 | 6.65 | 28.260 | +2.10 | 216.46 | +2.13 | 4.832 | +0.13 |

Tree size and depth barely move: node count changes by at most **+0.45 %** and
depth by at most **+2** levels (san-miguel 49 → 51). The knob changes **0.8 % to
6.8 %** of collapse decisions, rising with λ — so it is doing something, but it
is a refinement of the SAH collapse rather than a different tree.

---

## Conclusion: **mixed**

Directional collapse is not a general win, and it is not a general loss.

**Where it clearly helps.**

* **Intel Sponza** — the only scene that improves on *all three* views. At
  λ = 0.5 the trained view A drops **10.8 %** node visits and **12.4 %** box
  tests, and the held-out views still gain 3.3 % and 5.7 %. The effect saturates:
  λ = 0.5, 0.75 and 1.0 are within 0.5 pp of each other.
* **San Miguel view B** — the single largest effect anywhere, **−15.8 %** node
  visits and **−16.9 %** box tests at λ = 0.5. But this is a *held-out* view; the
  trained view A gets slightly **worse** on node visits (+1.4 %) while its box
  tests improve (−2.0 %). That is the opposite of the pattern the hypothesis
  predicts.

**Where it does nothing.** Hairball moves by under 1 % on the trained view and
changes only 0.8–2.6 % of decisions — expected, since its `W` is the closest to
isotropic (0.500, 0.713, 0.453) and an isotropic `W` provably reduces the weight
back to surface area.

**Where it hurts.** Bistro's trained view A is **worse at every nonzero λ**
(+2.6 % to +4.0 %), despite having the most anisotropic workload of all four
scenes (Wx = 0.944). Hairball view C is consistently ~+5 % worse.

**Three observations that matter more than the averages.**

1. **Training on a view does not reliably improve that view.** Two of four scenes
   get worse on the view their `W` was computed from. Whatever the weight is
   doing, it is not specialising the tree to the training direction in the way
   the hypothesis assumes.
2. **The effect is not monotone in λ and not stable across views.** Sponza
   saturates cleanly; bistro's view B swings from −1.4 % at λ = 0.5 to +7.1 % at
   λ = 1.0; san-miguel view B peaks at λ = 0.5 and falls back to −6.7 % at
   λ = 1.0. There is no λ that is good everywhere.
3. **Primitive work is essentially untouched** (mostly within ±1.3 %). The
   collapse changes which interior nodes exist, not which triangles get tested,
   so the entire effect is in node and box work.

**Recommendation.** The idea is not dead — a reproducible 10–16 % reduction in
node visits and box tests on two of four large scenes, at a cost of under 0.5 %
tree size and with exact traversal preserved, is a real effect worth
understanding. But as formulated it is not deployable: the sign of the effect is
not predictable from the scene or from `W`, and the trained view is not reliably
the one that benefits. The obvious next question is *why* Sponza and San Miguel
view B respond so strongly while bistro regresses — that is a diagnosis task on
four specific trees, not another sweep.

---

## Artifacts

`results/w8_dir_2026_09_01/w8_directional.csv` — 60 rows (4 scenes × 3 views ×
5 variants).

SHA-256: `ac990c93371b6d05ed139dbb0edbdcc5e5a8c42feb95c15a1326a5e32ac13f7d`. Every row records `run_id`, `git_commit`, scene,
triangle count, view, rayset hash, λ, node/box/prim per ray and their percentage
changes, tree node count and depth, changed-decision count and percentage, `W`,
and the correctness verdict with mismatch and tie counts.

**Not used, and no claim made:** CPU timing (recorded nowhere in this
experiment), GPU behaviour, widths 4 and 16, other ray distributions, EPO, and
any statistical machinery beyond the direct percentage changes above.
