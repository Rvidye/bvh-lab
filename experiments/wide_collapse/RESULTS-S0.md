# S0 — Phase 0 feasibility report

**Date:** 2026-09-01
**Run id:** `phase0_2026_09_01`
**Scope:** Phase 0 of `PLAN-directional-wide-collapse.md` only — feasibility
measurement, instrumentation and camera freeze. **Phases 1–4 were not started.**
No directional weight exists in this tree, no alternative wide tree was built,
and no performance comparison was made.

Base commit `fc3a28f528b55cbbb128b372a5601e52359ad12e`; the Phase-0 changes were
uncommitted at measurement time, so every row records `git_commit=fc3a28f`,
`dirty=1`.

---

## 1. Headline

Phase 0 did what it was for: **it falsified two of my own predictions and caught
one latent defect that would have silently corrupted every wide-tree result.**

| # | prediction (plan rev. 2/3) | measured | verdict |
|---|---|---|---|
| 1 | San Miguel width-8 emitted depth ≈ 21, stack ≈ 148 | **depth 49, stack 344** | prediction wrong; problem **worse** than predicted |
| 2 | legacy DP scratch (5.27 GB) "will not fit" | 5021 MB, allocation probe **succeeded**, peak RSS 6915 MB | prediction wrong; it **does** fit |
| 3 | camera screen accepts sensible interior views | **178 of 192 candidates rejected** by a mis-specified threshold | screen was wrong, corrected |

The one genuinely dangerous finding is #1, below.

---

## 2. The latent defect: silent traversal-stack overflow

`intersect()` and `occluded()` have **no bounds check** on `stack_size++`. A
width-`W` tree needs `1 + (W-1) * emitted_depth` entries. Measured requirements
against the constant that was in the tree before Phase 0 (`bvh2_stack_size = 128`):

| scene | w4 | w8 | w16 |
|---|---|---|---|
| intel-sponza | 61 | 99 | **166** |
| hairball | 70 | 113 | **211** |
| bistro | 73 | 120 | **196** |
| san-miguel | **163** | **344** | **721** |

**Nine of twelve configurations exceed 128.** San Miguel overflows at *every*
width, including width 4. Because there is no bounds check, the result would have
been silent memory corruption producing wrong hit records — not a crash, and not
something the oracle would necessarily catch, since the corruption is in the
traversal's own stack frame.

`bvh2_stack_size` is now **frozen at 1024** (max measured requirement 721, plus
headroom), and `required_stack_depth(width, emitted_depth)` plus a mandatory
per-tree check are in `bvh/build/collapse.h`. A tree that fails the check
produces no row.

**Design consequence worth carrying into Phase 1.** San Miguel's emitted depth
barely falls under collapse: 62 → 54 / 49 / 48 at widths 4 / 8 / 16, while node
count falls 19.94M → 13.91M / 11.93M / 11.03M. The DP compresses node *count*
well and depth hardly at all, because the tree contains long spindly paths that a
wide node cannot absorb. A 344-entry traversal stack at width 8 is far beyond
what a GPU kernel would budget, so San Miguel is a genuinely hard case for wide
traversal in this representation, independently of anything directional. This is
a finding about the scene and the existing collapse, not about the hypothesis.

---

## 3. Feasibility measurements

Single process, Release x64, AMD Ryzen 7 8745HX (8C/16T), 31.8 GB RAM.

| scene | triangles | load s | build s | binary nodes | binary depth | peak RSS MB | diagonal |
|---|---|---|---|---|---|---|---|
| intel-sponza.obj | 3,746,948 | 3.35 | 3.74 | 7,493,895 | 31 | 2506 | 1719 |
| hairball.obj | 2,880,000 | 2.23 | 3.31 | 5,759,999 | 34 | 1931 | 15.91 |
| bistro.obj | 2,829,873 | 2.80 | 3.09 | 5,659,745 | 35 | 1915 | 161.6 |
| san-miguel.obj | 9,971,513 | 11.51 | 11.06 | 19,943,025 | 62 | 6915 | 75.65 |

Load and build are **far cheaper than the plan's 1–8 min / 1–5 min estimates** —
San Miguel is 11.5 s and 11.1 s. The whole-matrix runtime estimate of 6–12 hours
should be revised down substantially once Phase 3 exists; scene setup is not the
bottleneck, the oracle is.

### Collapse, per width

| scene | w | wide nodes | emitted depth | fullness | collapse s | scratch MB | legacy MB | reduction | req. stack |
|---|---|---|---|---|---|---|---|---|---|
| intel-sponza | 4 | 5,232,781 | 20 | 3.52 | 0.62 | 343 | 1887 | 5.50× | 61 |
| intel-sponza | 8 | 4,484,656 | 14 | 6.08 | 0.77 | 572 | 1887 | 3.30× | 99 |
| intel-sponza | 16 | 4,141,627 | 11 | 10.49 | 1.16 | 1029 | 1887 | 1.83× | 166 |
| hairball | 4 | 3,985,854 | 23 | 3.60 | 0.58 | 264 | 1450 | 5.50× | 70 |
| hairball | 8 | 3,409,544 | 16 | 6.44 | 0.64 | 440 | 1450 | 3.30× | 113 |
| hairball | 16 | 3,155,558 | 14 | 11.45 | 1.00 | 791 | 1450 | 1.83× | 211 |
| bistro | 4 | 3,963,397 | 24 | 3.50 | 0.57 | 259 | 1425 | 5.50× | 73 |
| bistro | 8 | 3,401,720 | 17 | 5.95 | 0.63 | 432 | 1425 | 3.30× | 120 |
| bistro | 16 | 3,147,968 | 13 | 9.90 | 0.97 | 777 | 1425 | 1.83× | 196 |
| san-miguel | 4 | 13,913,523 | 54 | 3.53 | 2.11 | 913 | 5021 | 5.50× | 163 |
| san-miguel | 8 | 11,927,443 | 49 | 6.10 | 2.22 | 1522 | 5021 | 3.30× | 344 |
| san-miguel | 16 | 11,025,714 | 48 | 10.46 | 3.45 | 2739 | 5021 | 1.83× | 721 |

All twelve now pass the stack bound against the frozen 1024.

Collapse is **cheap** — 0.6–3.4 s. The plan's 5–30 s estimate was pessimistic, so
the per-λ variant cost is not a constraint.

### DP scratch: the fix is worth having, but it was not a blocker

The scratch table is now sized to the requested width rather than to
`max_collapse_width`, giving 5.50× / 3.30× / 1.83× reductions at widths 4 / 8 /
16. **However, my revision-2 claim that the legacy 5.27 GB allocation "will not
fit" was wrong.** An explicit contiguous allocation probe of the legacy size,
taken with the mesh and the binary tree already resident, **succeeded on every
scene**, including San Miguel's 5021 MB (peak RSS 6915 MB against 31.8 GB
installed). The fix is retained because it is strictly better and reduces peak
RSS materially, but it is a convenience, not a prerequisite. Reported here
because the plan asserted otherwise.

### Oracle throughput

| scene | 1 thread (M tri-tests/s) | 16 threads | speedup |
|---|---|---|---|
| intel-sponza | 68.5 | 576.3 | 8.4× |
| hairball | 53.9 | 411.4 | 7.6× |
| bistro | 48.6 | 392.1 | 8.1× |
| san-miguel | 50.2 | 297.7 | 5.9× |

Scaling is sub-linear (5.9–8.4× on 16 threads), as expected for a
memory-bandwidth-bound linear scan. San Miguel scales worst, consistent with its
working set.

---

## 4. Frozen: oracle sample

Budget: **60 minutes for the whole matrix**. Seventeen distinct raysets per scene
(primary × 3 views; shadow_ao and diffuse_1 × 3 views × 2 seeds; incoherent × 2
seeds), four scenes.

| scene class | rays per rayset | per block (256 blocks) | hit / miss quota |
|---|---|---|---|
| ≤ 5M triangles | **7168** | 28 | 14 / 14 |
| > 5M triangles | **1536** | 6 | 3 / 3 |

Estimated total at the measured 16-thread throughputs: **3399 s = 56.7 min**,
inside budget. Both sizes divide 256 exactly, so every block carries an equal
quota and the 50/50 hit/miss split is exact.

This replaces the previous experiment's 16-ray screen, which was indefensible.

---

## 5. Frozen: cameras

The plan's screen was `hit_fraction ∈ [0.55, 0.98]`, median hit distance ≥ 5% of
the scene diagonal, and pairwise view separation ≥ 45°.

**The upper hit-fraction bound was mis-specified and is removed.** Measurement
showed it rejected **178 of 192** candidates across the first three scenes, with
hit fraction exactly 1.000 at the 25th percentile. These are enclosed interiors;
full coverage is the correct and desirable condition, not the "wall filling the
frame" defect the bound was written to catch. Its actual purpose is now served by
a test that measures the right thing:

* `hit_fraction >= 0.55` (unchanged)
* `median_hit_over_diagonal >= 0.05` (unchanged)
* **new** `p10_hit_over_diagonal >= 0.005` — rejects a camera pressed against a
  surface, which is what the upper bound was a poor proxy for
* pairwise view-direction separation `>= 45°` (unchanged)
* **new** pairwise eye separation `>= 0.05 × diagonal` — three views from the same
  spot would not be three views

This is a deviation from the plan's §8, made during Phase 0 **before any
performance number existed**, and recorded here rather than applied silently.

Candidates are deterministic: eye placed `0.004 × diagonal` off the surface of
triangle `(7919·i + 13) mod N` on the side facing the scene centre, looking at
the centroid of triangle `(104729·i + 12345) mod N`, screened with a 64×64
primary trace. `camera::frame_bounds` was deliberately **not** used — it places
the eye outside the bounding sphere, which for interior architectural scenes
frames the outside of a building.

Result: 64 candidates per scene, and three well-separated views frozen for each.

| scene | accepted | view A | view B | view C |
|---|---|---|---|---|
| intel-sponza | 30 | cand 0, hf 1.000 | cand 1, hf 1.000 | cand 6, hf 1.000 |
| hairball | 4 | cand 0, hf 0.606 | cand 6, hf 0.796 | cand 37, hf 0.993 |
| bistro | 22 | cand 1, hf 1.000 | cand 28, hf 0.998 | cand 31, hf 0.945 |
| san-miguel | 25 | see `cameras.csv` | | |

Full parameters — position, target, up, focal, direction, hit fraction, median
and p10 hit distance, scene diagonal, and the frozen oracle sample size — are in
`experiments/wide_collapse/cameras.csv`.

Hairball's 4 accepted candidates is the thinnest margin; it is a hairball in open
space, so many candidate views see substantial background. Three separated views
were still obtained.

---

## 6. Instrumentation added, and proof it is inert

Added to `trace_stats` and `null_stats`, called from `traverse_bvh2.h`:

* `box_hits` — children whose box test passed, i.e. entries pushed;
* `pruned_pops` — entries popped and discarded because the incumbent already beat
  them (closest-hit only).

Together with the existing counters these give the vocabulary of the plan's §5:
interior visits (`node_steps`), child slots examined (`box_tests`), primitive
tests (`tri_tests`), leaf visits (`prim_steps`), stack pushes (`box_hits`), wasted
stack traffic (`pruned_pops`) and peak occupancy (`max_stack`). Logical child
bytes is derived, not measured, and remains labelled as a model.

**Inertness proof.** The `fc3a28f` tree was materialised with `git archive`,
built in the scratchpad, and run against the Phase-0 binary on the identical
command line including `--wide` (which exercises the DP collapse at widths 4 and
8). Comparing every deterministic column:

| artifact | rows | deterministic columns | differences |
|---|---|---|---|
| `m1_bvh2.csv` | 5 | 29 | **0** |
| `m1_workload.csv` | 24 | 21 | **0** |
| `m3_wide.csv` | 30 | 30 | **0** |
| `m3_depth_profile.csv` | 45 | 14 | **0** |
| 9 PNG renders | — | byte comparison | **all identical** |

**2179 deterministic cells, zero differences.** So the counter additions, the
stack-size change and the DP scratch restructuring change no behaviour.

Full suite: **238 tests, 238 passed**, Release x64 build with **0 warnings, 0
errors**.

---

## 7. Files changed

| file | why |
|---|---|
| `bvh/core/trace_stats.h` | `box_hits`, `pruned_pops` counters (additive) |
| `bvh/core/traverse_bvh2.h` | call the new counters; `bvh2_stack_size` 128 → **1024**, frozen from measurement |
| `bvh/build/collapse.h` | `required_stack_depth()`; `collapse_report` gains `scratch_bytes`, `legacy_scratch_bytes`, `required_stack`, `stack_bound_ok` |
| `bvh/build/collapse.cpp` | DP scratch sized to `width + 1` instead of `max_collapse_width + 1`; report the new fields. No decision logic touched |
| `bvh/util/memory.{h,cpp}` | peak/current working set and a contiguous-allocation probe, for feasibility measurement only |
| `trax/phase0.{h,cpp}` | the Phase-0 probe: timings, memory, collapse per width, camera screening, oracle throughput |
| `trax/main.cpp` | `--phase0` and its options; additive and gated, exactly like `--direction_d` |
| `bvh/bvh.vcxproj{,.filters}`, `trax/trax.vcxproj{,.filters}` | registration |
| `PLAN-directional-wide-collapse.md` | revision 4: V7 excluded, the two later-phase corrections recorded, oracle sample frozen |
| `experiments/wide_collapse/cameras.csv` | the frozen camera set |

Not touched: the binary SAH builder, `bvh2_node`, the traversal *decisions*, the
rayset generator, `quality`, and every Direction D file.

---

## 8. Artifacts

| path | bytes | SHA-256 |
|---|---|---|
| `results/phase0_2026_09_01/phase0_feasibility.csv` | 1,453 | `77abab4402dab90bd26ff0f6e3abf1d5740873e1c982df6d0b304c1e096d7228` |
| `results/phase0_2026_09_01/phase0_collapse.csv` | 2,691 | `bc47f0d60cc1ac7fff36d9971da03b7f3febaa850d8c74224898126849130e13` |
| `results/phase0_2026_09_01/phase0_cameras.csv` | 59,098 | `6e09ecc4e1b980706c48c9a779af6f5e79059b248b69f9ae6a881d034fc4d221` |
| `experiments/wide_collapse/cameras.csv` | 2,852 | `9aac6ce0f40ad2842e90d10a0cc58d98c94100abbc0ae6d94d9b076b2e450c1f` |

All four are committed. `results/.gitattributes` already stores `results/**` CSVs
`-text`, so these hashes survive a checkout.

---

## 9. Deviations from the plan, recorded

1. **Camera screen**: upper hit-fraction bound removed, p10 near-field test and
   eye-separation test added (§5). Made before any performance number existed.
2. **`bvh2_stack_size` frozen at 1024**, not the 256 written into revision 3 —
   256 was itself only a placeholder, and the measurement showed 721 is required.
3. **Two revision-3 predictions were wrong** and are corrected in §2 and §3: the
   San Miguel emitted depth (much worse than predicted) and the legacy scratch
   allocation (fits, contrary to the claim that it would not).
4. The plan's Phase-0 item "parallelised stratified sampled oracle" is delivered
   as the **throughput benchmark plus the frozen sample specification**; the
   sampler itself is Phase 3 work, since nothing in Phase 0 validates a variant.

## 10. Stop

**S0 reached. Phase 1 has not been started** and will not be until this report is
reviewed. No directional code, no alternative wide trees, no performance
comparison, and V7 remains excluded.
