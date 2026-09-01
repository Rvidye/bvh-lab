# Direction D large-scene screening — simple report

## Plain answer

We should **not stop investigating direction-aware BVH construction**, but we
should **not use the current projected-triangle score as the builder objective
unchanged**. On these four large scenes it carries a real directional signal,
but it gives almost no extra information beyond a simple non-directional
surface-density score.

## What this run tested

This run did not build a directional tree. It kept the ordinary SAH tree and
asked whether the directional score could identify the more useful of two
existing child subtrees. The result is therefore a screening test for one
possible input signal, not a speed or tree-quality result.

## Results in plain language

| Scene | Result | Meaning |
|---|---|---|
| Hairball | Encouraging | Direction helped consistently, but its advantage over the simple baseline was small. |
| Bistro | Encouraging | Direction helped in most workloads, but again by only a small amount. |
| San Miguel | Mixed | Direction helped some workloads and hurt others; the overall advantage was very small. |
| Intel Sponza | Weak | Direction lost overall, and it was particularly poor for the camera rays. |

Across all four scenes, the directional score and the simple baseline were
effectively tied: about **54.2% versus 54.0%**. Only Hairball and Bistro reached
the experiment's basic predictive target; the plan required three scenes. The
extra advantage over the baseline was about **0.2 percentage points**, far
short of the planned **3-point** requirement.

## What the result means

The current score is not random: larger score values generally correspond to
fewer empty box hits. However, most of that information was already available
from ordinary surface density. That makes this score too weak to justify
rewriting the builder around it by itself.

This does **not** show that direction-aware trees are a bad idea. It only shows
that the current triangle-projection ratio is not a strong standalone guide for
building them. No directional tree was created in either the original run or
this large-scene run, so neither run measured the outcome we ultimately care
about.

## Recommended next experiment

Build and compare three real trees:

1. the current SAH tree;
2. a tree built or collapsed using exact direction-conditioned AABB projected
   area and representative training rays;
3. a hybrid tree that combines ordinary SAH with the directional term.

Then test them on both the training directions and held-out directions. Compare
actual node visits, triangle tests, memory traffic, tree size, and runtime. That
experiment directly answers whether directional information creates a better
tree.

The current projected-triangle score may remain as a secondary hit or
termination hint, but it should not be the sole split or collapse cost.

## Confidence and limitations

- The scenes contain roughly 2.8–10 million triangles, and all 28 published
  scene/workload rows met the experiment's support threshold.
- All 238 unit tests passed after adding large-scene support.
- Sixteen evenly distributed rays from every published workload were checked
  against the brute-force oracle; every sampled ray matched. A full brute-force
  check of every ray was impractical at this scene scale.
- San Miguel required increasing the traversal stack from 64 to 128 entries
  because its ordinary SAH tree reaches depth 62. This changed capacity, not
  the tree or traversal decisions.
- The screen began at 128×128 rays and automatically increased low-support
  workloads to 1024×1024. It is a diagnostic screen rather than the frozen
  formal 512×512 experiment.
- The local OBJ files do not record trustworthy upstream URLs or licenses, so
  they remain development scenes and cannot make the original gate formal.
- Missing-material warnings do not affect this geometry-only experiment.

## Artifacts

Run: `results/d1_large_sampled_2026_08_31`

| File | SHA-256 |
|---|---|
| `d1_summary.csv` | `c08c8daf49034b325ea89013dd04176e3edc61d8d9c7ff5f2db1fb7ae87192b1` |
| `d1_directional_bins.csv` | `69466c8bdb182f00e7c0fc12c522a9d4306f9a5e5285d08515b8247a9249b1d0` |
| `d1_directional_pairs.csv` | `08c4854d351a062117a12acd7ce6bad4c7e22c9a4161ee50552a50eeb92ad04e` |
| `d1_directional_pairs.csv.gz` | `1e0a0a6b6ad8dfb89d26076c65568aead62dff7079e9d9722458f5c3583cdfcd` |

The deterministic aggregator rechecked every CSV reconciliation identity and
reported no failures.
