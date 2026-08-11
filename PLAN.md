# bvh-lab — Design & Implementation Plan

A minimal, measurement-first BVH research lab.
Structure modelled on **FoxRT/RwRT**; code style and data layout modelled on **Arches/rtm**.

Paper study notes live in `C:\data\HWRT-Papers\BVH-STUDY-PLAN.md`.

---

## 1. Design patterns

### 1.1 What we take from where

The two codebases are good at different things, and they split cleanly:

| Concern | Model | Why |
|---|---|---|
| Project/solution layout, module split, test project | **FoxRT** | pbrt-v4 lineage; proven for exactly this shape of codebase |
| Diagnostics, stats, assertions | **FoxRT** | `STAT_COUNTER` + `CHECK`/`DCHECK` are zero-friction and already battle-tested |
| Umbrella header, include style | **FoxRT** | `<util/log.h>` angle-bracket + `$(ProjectDir)` on the include path |
| Code style within a file (naming, flow) | **Arches** | snake_case, tight explicit loops, no ceremony |
| Data layout, POD structs, bitfields, alignment | **Arches** | built for a hardware simulator; already device-shaped |
| Host/device separation | **Arches** | `#ifndef __riscv` is the same problem as our CPU/CUDA split |
| Generic traversal | **Arches** | `template<typename N, typename P>` over node and prim type |
| Layout taxonomy, mutable-vs-compact split | **tinybvh** | `BVHBase` + per-layout subclasses; `BVH_Verbose` for optimization |
| Benchmark harness, ray distributions | **tinybvh** | `Experiment` / `RayDistribution` is the harness we specced, already built |
| GPU kernel organisation | **tinybvh** | one kernel file per layout; no generic GPU traversal |
| Byte-exact struct tuning | **bvh_article** | Ray = 64 B, Intersection = 16 B, Node = 32 B, on purpose |
| Binned-SAH split details | **bvh_article** | bins over *centroid* bounds, tracked separately from node bounds |

### 1.2 Patterns adopted from FoxRT

**P1 — Three projects: StaticLibrary + Application + test.**
`RwRT` (StaticLibrary) / `FoxRT` (Application) / `RwRT_Test` (gtest) becomes
`bvh` (StaticLibrary) / `trax` (Application) / `bvh-test` (gtest), plus `trax-cuda` later.

**P2 — Umbrella header carries types and forward declarations only.**
[`RwRT.h`](../../data/FoxRT/RwRT/RwRT.h) is 100 lines of `using Float = float`, forward
declarations, and nothing else. Ours is `bvh.h`: `using f32`, `uint`, forward decls for
`Mesh`, `BVH2`, `WBVH`, `Ray`, `Hit`, and the `BVH_DEV`/`BVH_HOST` qualifiers. No logic,
no heavy includes. Every other header starts with `#include <bvh.h>`.

**P3 — `$(ProjectDir)` on the include path; angle-bracket includes throughout.**
RwRT sets `AdditionalIncludeDirectories = $(SolutionDir)ext\;$(ProjectDir);...` which is why
every file says `#include <util/log.h>` and never `#include "../util/log.h"`. Do the same.
Relative-path includes are how a header tree becomes unmovable.

**P4 — One module = one `.h`/`.cpp` pair = one test file.**
`util/` in RwRT is 30 leaf modules each doing one thing. Mirror that discipline; it's what
makes the tree navigable without a map.

**P5 — Self-registering thread-local counters.**
[`util/stats.h`](../../data/FoxRT/RwRT/util/stats.h) is the pattern to copy for host-side
instrumentation:
```
#define STAT_COUNTER(title, var)                                   \
  static thread_local int64_t var;                                 \
  static StatRegisterer STATS_REG##var([](StatsAccumulator& a) {   \
    a.ReportCounter(title, var); var = 0; });
```
Declare a counter next to the code it measures, `++var` in the hot path, and it shows up in
the report with no plumbing. Take `STAT_COUNTER`, `STAT_RATIO`, `STAT_PERCENT`, and
`STAT_INT_DISTRIBUTION` (the last gives sum/count/min/max, which is most of what we want for
per-ray node counts). Skip the pixel-stats machinery for now.

**P6 — `CHECK` / `DCHECK` split.**
[`util/check.h`](../../data/FoxRT/RwRT/util/check.h): `CHECK_*` always on, `DCHECK_*` compiled
out unless `RWRT_DEBUG_BUILD`. We use `BVH_DEBUG_BUILD`, defined only in the Debug config.
Hot-path invariants (stack depth, child index bounds, quantization round-trip) go in `DCHECK`
so Release stays clean.

**P7 — Preprocessor and language settings, copied verbatim.**
`LanguageStandard = stdcpp20`, and `NOMINMAX;_CRT_SECURE_NO_WARNINGS;_ENABLE_EXTENDED_ALIGNED_STORAGE`
in every config. `NOMINMAX` in particular — without it `<windows.h>` breaks every `min`/`max`
you write.

### 1.3 Patterns adopted from Arches

**P8 — One header, two compilation targets, host-only code fenced off.**
Arches wraps every builder in `#ifndef __riscv` so the same header serves the host builder
and the bare-metal RISC-V kernel ([`bvh.hpp:90`](../../dev/arches/include/rtm/bvh.hpp:90)
opens the fence, `:1014` closes it). Our version:

```
core/           BVH_DEV, POD only, no STL/GLM/virtual/exceptions.
                Compiles as host C++20 and as CUDA __host__ __device__.
build/ encode/  host only. STL, GLM, threads, file I/O — all fine.
```
The rule that makes it work: **`core/` never allocates and never owns.** It takes
`const Node*` + counts. Ownership lives in host containers.

**P9 — POD nodes, cache-line aligned, bitfield pointers.**
```
union BVHPtr { struct { uint32_t is_int:1, prim_cnt:5, prim_idx:26; }; uint32_t raw; };
struct alignas(32) Node { AABB aabb; BVHPtr ptr; };
```
[`bvh.hpp:20`](../../dev/arches/include/rtm/bvh.hpp:20). Two things to keep: `raw` in the
union (lets you compare/dedupe pointers in one compare — used all over the collapse code),
and `alignas` matched to the real cache line, because the whole memory-traffic axis depends
on node size being exactly what you think it is. `static_assert(sizeof(Node) == 32)`.

**P10 — Struct-of-arrays mesh, reorder once after build.**
Separate `vertices` / `vertex_indices` / `normal_indices` / `material_indices`. The builder
permutes primitives, then [`_reorder`](../../dev/arches/include/rtm/bvh.hpp:251) applies the
permutation to the index arrays once, so leaves are contiguous ranges and traversal never
indirects through a primitive-id table.

**P11 — Parent-before-children index invariant.**
Every builder must emit nodes such that `child_idx > parent_idx`. Then refit is a single
reverse linear scan ([`_refit`](../../dev/arches/include/rtm/bvh.hpp:267)) and is trivially
parallelisable by level. Free to maintain, expensive to retrofit. `DCHECK` it after every build.

**P12 — Traversal generic over node and primitive type.**
[`intersect.hpp:193`](../../dev/arches/src/trax-kernel/intersect.hpp:193):
`template <typename N, typename P> intersect(const N* nodes, const P* prims, ...)`.
We extend this with an explicit `Codec` concept and a `Stats` parameter (§2).

**P13 — Stats printed as ASCII histograms.**
[`_print_stats_wbvh`](../../dev/arches/include/rtm/bvh.hpp:905) prints node/leaf fullness as
dot-bars. Cheap, and it catches broken collapses instantly — a width-8 collapse that produces
90% 2-child nodes is visible at a glance and invisible in a mean.

**P14 — Serialize built BVHs with a version stamp.**
[`_serialize`/`_deserialize`](../../dev/arches/include/rtm/bvh.hpp:976) with a `VERSION`
constant that invalidates the cache on layout change. You will rebuild Sponza several hundred
times over the course of this project.

### 1.4 Patterns adopted from tinybvh and bvh_article

**P15 — Layout is a class; every layout carries its own `SAHCost`, `Save`/`Load`, and a
converter.**
tinybvh's `BVHBase` has subclasses `BVH`, `BVH_Double`, `MBVH<M>`, `BVH4_CPU`, `BVH8_CPU`,
`BVH_GPU`, `BVH4_GPU`, `BVH8_CWBVH` ([tiny_bvh.h:894–1618](../../dev/tinybvh-main/tiny_bvh.h)).
Independent confirmation of the builder × layout axis split in §2.1. We use free functions and
templates instead of inheritance (no vtables in `core/`), but the taxonomy is the same.

**P16 — Three forms of the binary BVH, not two.** *This changes §2.2.*
tinybvh separates compact `BVH` (32 B nodes, traversal) from `BVH_Verbose`
([tiny_bvh.h:1330](../../dev/tinybvh-main/tiny_bvh.h)) — a fatter mutable form with parent
pointers and a node freelist, used only by optimization passes. Optimization (rotations,
reinsertion, treelets) needs parent links and node deletion; traversal wants 32 bytes and no
back-pointers. Forcing one struct to do both makes both worse. So:

```
bvh2_verbose   mutable, parent pointers, freelist   → optimization passes
bvh2           compact 32 B, parent-before-child    → traversal, refit, layout source
wbvh/cwbvh/hec immutable compiled layouts           → traversal only
```
`bvh2_verbose` arrives with M2 (maintenance), but `bvh2` must be
convertible to and from it. That costs nothing now and unblocks the whole quality axis later.

**P17 — EPO is already implemented; take it.**
`EPOCost`, `EPOArea`, and `W_EPO = 0.71f` live at
[tiny_bvh.h:1032–1079 and :3239](../../dev/tinybvh-main/tiny_bvh.h). This is the
Aila/Karras/Laine 2013 metric flagged as the single most important gap in the paper study —
we now have a working reference, so EPO belongs in the metrics module from **M1** rather than
staying aspirational.

**P18 — The benchmark harness shape.**
[`benchmark/experiment.h`](../../dev/tinybvh-main/benchmark/experiment.h):
`Experiment(layout, buildFlags, scene, raySet, flags, view)` → `Run()`, split into
`RunBuildExperiment()` / `RunTraceExperiment()`, with per-layout GPU runners and cached
prim/ray sets. [`benchmark/ray_distribution.h`](../../dev/tinybvh-main/benchmark/ray_distribution.h)
makes ray sets first-class: an enum (`PRIMARY_VIEW1..3`, `FIRST_BOUNCE`, `SECOND_BOUNCE`,
`AO_RAYS`) plus **SoA** storage (separate `O`, `D`, `tmin`, `tmax` arrays — GPU-ready). Copy
both shapes directly; this is invariant I4 already built.

**P19 — One GPU kernel file per layout. No generic GPU traversal.**
`kernels/traverse_bvh2.cl`, `traverse_bvh4.cl`, `traverse_cwbvh.cl`, `wavefront.cl`. Settles
the M5 question: don't attempt one templated device traversal across layouts.

**P20 — The CWBVH hot loop needs specific intrinsics.**
[`traverse_cwbvh.cl:1–70`](../../dev/tinybvh-main/kernels/traverse_cwbvh.cl) shims
`__activemask`, `__bfind`, `__popc` and the three-operand `fmin_fmin`/`fmax_fmax` (Aila–Laine
VMIN/VMAX), with inline PTX under `#ifdef ISNVIDIA`. **In CUDA these are all native**, so our
M5 port is easier than tinybvh's OpenCL shim for `__popc` and `__activemask`, which ARE CUDA
built-ins, and no easier for `__bfind` and the three-operand min/max, which are not (see §7).
Useful as a checklist of
what the algorithm actually requires: octant-indexed child ordering, `__bfind` for
next-child selection, `__popc` for the child-offset prefix.

**P21 — Byte-exact struct tuning, stated as intent.**
[`bvh_article/bvh.h`](../../dev/bvh_article-main/bvh.h): `Ray` is exactly 64 B (O, D, rD, hit),
`Intersection` exactly 16 B with `instPrim` packing instance (12 bit) + primitive (20 bit),
`BVHNode` exactly 32 B by unioning `leftFirst`/`triCount` into the AABB's w lanes, `Tri` 64 B
with the centroid precomputed. Every one of these carries a comment saying so. We do the same
and back it with `static_assert(sizeof(x) == N)` — the memory-traffic axis is meaningless if
node size isn't exactly what we think.

**P22 — Bin over centroid bounds, not node bounds.**
[`bvh_article`](../../dev/bvh_article-main/bvh.h) threads `centroidMin`/`centroidMax` through
`Subdivide` separately from the node AABB and bins over *those*. This is Wald 2007 as written.
Arches' [`_split_sah_binned`](../../dev/arches/include/rtm/bvh.hpp:359) bins over the union of
primitive AABBs instead, which spreads bins over a wider range than the centroids occupy and
yields measurably worse splits. Use centroid bounds. Also note bvh_article defaults to
`BINS = 8` where Arches uses 32 — make it a `constexpr` parameter and measure.

### 1.5 Conventions

- **snake_case** for everything — types, functions, members. Arches style. It reads better for
  math and matches what `core/` will look like in CUDA.
- Header guards: `#pragma once` (Arches). Simpler than FoxRT's `BVH_UTIL_X_H`, and MSVC/nvcc
  both handle it.
- Namespace: `bvh` for the library, no nesting.
- `// section` comments in the FoxRT style (`// bvh2 public methods`) to break up long headers.
- Private members prefixed `_` (Arches).
- No exceptions anywhere. `CHECK` on programmer error, error codes / `bool` on I/O failure.

### 1.6 Anti-patterns — explicitly avoid

- **The `BuildArgs` mega-struct.** [`bvh.hpp:70`](../../dev/arches/include/rtm/bvh.hpp:70)
  has 11 interacting flags whose valid combinations are undocumented, plus a `MaxPrims` enum
  that smuggles a type tag in the high bits (`PAIR = 0x20+2`). It's why the Arches builder is
  hard to read. Separate functions, explicit parameters, template policies.
- **Runtime type dispatch on the hot path.** Codec and layout are template parameters, not
  enum switches inside the traversal loop.
- **Virtual interfaces in `core/`.** FoxRT can afford `Primitive*` polymorphism; device code
  cannot.
- **Dead configuration.** Arches' `PAIR` path is declared, printed in stats, and never set by
  any caller — and the code inside it is wrong (it ignores the `prim_id0` offset and bounds
  `j` by `prim_cnt` instead of 3). If a config isn't exercised by a test, delete it.
- **One project per stage.** bvh_article ships ten `.vcxproj` files — `basics`, `faster`,
  `quickbuild`, `animation`, `toplevel`, … — one per article, each a standalone program. That
  is right for a tutorial series and wrong for us: it duplicates the shared code ten times.
  Keep the *discipline* (every stage is independently runnable and produces a picture) but
  express it as CLI configurations of one `trax` binary, not as separate projects.

---

## 2. Architecture

### 2.1 Six orthogonal axes

The previous plan named four axes and then, in the schedule, collapsed them into one chain
(`BVH2 → CWBVH → H-PLOC → HECWBVH`). That chain mixes independent concepts: CWBVH is a
*layout*, H-PLOC is a *builder*, and the two are not sequential steps. The roadmap is now
organised around the axes themselves, and a milestone advances one axis at a time.

| Axis | Values |
|---|---|
| **1. Builder** | sweep SAH (reference) · binned SAH (practical) · LBVH · H-PLOC · SBVH (optional high-quality reference) |
| **2. Topology / layout** | BVH2 · uncompressed BVH4/BVH8 · CWBVH · HEC/HE2 block layouts |
| **3. Maintenance** | rebuild · refit · refit+rotations · periodic rebuild · quality-triggered rebuild · partial rebuild / dirty-block re-encode |
| **4. Traversal** | scalar CPU · SIMD CPU · GPU · persistent threads · Arches/hardware |
| **5. Primitive / leaf** | indexed triangles · precomputed (Woop) triangles · fixed primitive blocks · merged/shared blocks |
| **6. Workload** | primary · shadow/AO · reflection · diffuse secondary/tertiary · coherent vs incoherent · static / deforming / instanced |

A result is only interpretable if it names its position on all six. See §9.

**The representation that carries the axes.** Builders emit `bvh2`; layouts are functions
`bvh2 → X`; codecs own leaf storage and leaf intersection; workloads are serialized ray sets.

Escape hatch, and it is load-bearing rather than hypothetical: **H-PLOC ships its own binary →
n-wide conversion in a single kernel launch** (H-PLOC §1), and fused collapsing exists
specifically to skip the binary intermediate. `bvh2` is the default path, not a mandatory one,
and the layout interface must not assume it.

### 2.2 Representations, and what refit can actually do

*Corrects an overstatement in the previous plan.* "Compressed layouts cannot be refit in
place" is too strong and hides the interesting engineering. Separate four distinct operations:

| Operation | What changes | Cost |
|---|---|---|
| **Geometry-bound refit** | leaf bounds follow moved vertices; topology fixed | cheap, one reverse scan |
| **Topology maintenance** | rotations / reinsertion change parent-child links | moderate; needs parent pointers |
| **Compressed-node re-encode** | a changed child bound may force re-deriving the node's shared anchor/exponent and requantising *all* sibling slots | per-node, local |
| **Primitive-payload regeneration** | the *representation-specific* primitive data must be rebuilt from the moved vertices, even when topology, leaf membership and encoding feasibility are all unchanged. CWBVH Woop triangles are a precomputed affine transform per triangle; HEC/HE2 FTB blocks hold quantised vertex payloads. Both are stale the moment a vertex moves. | O(primitives), and easy to forget |
| **Merged-block repair** | leaf membership or encode feasibility changed, so a merged primitive block is no longer valid and must be repaired, split, or dissolved | can cascade |

So a compressed node *can* be updated in place when the new child bounds still fit the
existing anchor/exponent, and requires local re-encoding when they do not. Only membership
changes force block repair. Quantifying where those boundaries fall is Research Direction B
(§11) — it is a question to measure, not an assumption to bake in.

**Payload regeneration is the one that gets forgotten.** It is not a bounds operation at all,
so it does not appear in any refit discussion, yet for a deforming mesh it is O(primitives)
work that a plain `bvh2` refit never has to do. That asymmetry can move the refit-versus-
rebuild crossover on its own, so it gets its own timer column (§6.4) rather than being folded
into "encode".

Three representations, per P16:

| Form | Shape | Used for |
|---|---|---|
| `bvh2_verbose` | fat, parent pointers, node freelist | optimisation: rotations, reinsertion, treelets |
| `bvh2` | compact 32 B, parent-before-child invariant | traversal, refit, source for every layout |
| `wbvh` / `cwbvh` / `hec` | compiled | traversal; re-encode rather than rebuild where possible |

`bvh2_verbose` arrives with M2 (maintenance), not later — moving dynamic scenes earlier is the
single biggest ordering change in this revision.

### 2.3 Primitive codec

A leaf format touches exactly three places: build feasibility/cost, encode, traverse.

```
struct ftb_codec {
    static constexpr uint MAX_PRIMS = 8;
    static constexpr addressing ADDRESSING = addressing::block;
    using block = ftb;

    // build (host)
    static bool  fits(const mesh&, uint begin, uint count);
    static float leaf_cost(const mesh&, uint begin, uint count);
    // encode (host)
    static bool  encode(const mesh&, uint begin, uint count, block& out);
    // traverse (device)
    template<typename Stats>
    BVH_DEV static void intersect_leaf(const block&, const ray&, hit&, Stats&);
    // validation (host)
    static uint  decode(const block&, triangle out[MAX_PRIMS], uint ids[MAX_PRIMS]);
};
```

`fits` + `leaf_cost` are what let a leaf format participate in the *build* rather than being a
post-hoc repack — Arches does this correctly in principle at
[`_cost_leaf`](../../dev/arches/include/rtm/bvh.hpp:493), called from inside the collapse DP.

**`intersect_leaf` is the device entry point, not `decode`.** Woop-transformed triangles
(CWBVH, Aila–Laine) intersect via a precomputed affine transform with no vertex fetch; forcing
them through `decode → triangle → Möller–Trumbore` discards the entire point of the format and
produces plausible-looking wrong numbers. `decode` exists only for host-side validation.

**Leaf addressing is the real coupling**, and it's a three-value enum:

| Mode | Node stores | Used by |
|---|---|---|
| `range` | (offset, count) into a global index buffer | classical bvh2, CWBVH |
| `block` | index into a fixed-size block array | FTB, DGF |
| `inline_` | leaf data *is* the node slot | HECWBVH merged nodes |

`static_assert(layout::LEAF_ADDRESSING == codec::ADDRESSING);` — invalid pairs fail at compile
time instead of rendering garbage.

**Do not build this abstraction before the second codec exists.** Rule of three: `bvh2` keeps
its hardcoded indexed-triangle leaves through M3; the interface gets extracted in M4 when
CWBVH introduces a genuinely different leaf. The one exception — make the collapse DP take the
leaf policy as a template parameter from the start (M3), because the leaf cost threads through
every entry of the DP cost table and retrofitting it is genuinely painful.

---

## 3. Solution layout

Existing files are marked ✅; planned files are unmarked. Keeping the distinction visible stops
the plan drifting away from the tree.

```
bvh-lab/                         (git root, = solution dir)
├─ bvh-lab.sln ✅   common.props ✅   PLAN.md ✅
├─ external/ ✅                  tinyobjloader, stb, googletest (source)
├─ bvh/                          → StaticLibrary ✅
│   bvh.h ✅                     umbrella: types + forward decls (P2)
│   core/                        ── BVH_DEV, POD, no STL ──
│     qualifiers.h ✅ vec.h ✅ aabb.h ✅ ray.h ✅ isect.h ✅ rng.h ✅
│     trace_stats.h ✅
│     bvh2.h            32 B node, rtm::BVH::Node layout
│     traverse_bvh2.h   ordered traversal, templated on codec + stats
│     wbvh.h  cwbvh.h  hec.h            (M3 / M4 / M8)
│     codec_indexed.h  codec_woop.h  codec_block.h
│   build/
│     bvh2_builder.{h,cpp}   sweep + binned SAH, median control
│     refit.{h,cpp}          M2      rotate.{h,cpp}       M2
│     collapse.{h,cpp}       M3      morton.{h,cpp}       M6
│     radix_sort.{h,cpp}     M6      lbvh.{h,cpp}         M6
│     hploc.{h,cpp}          M6      fused_collapse.{h,cpp} M7
│   encode/  cwbvh.{h,cpp} M4   hec.{h,cpp} M8
│   eval/
│     quality.{h,cpp}   SAH, EPO, overlap, occupancy, MSAH (M8)
│     trace.{h,cpp}     instrumented + timed traversal
│     rayset.{h,cpp}    M0  generate/serialize the six ray distributions
│     harness.{h,cpp}   M0  the (scene × … × device) matrix runner
│   util/   log ✅ check ✅ stats ✅ metrics ✅ timer ✅ mesh ✅ camera ✅
│           image ✅ parallel ✅ args.{h,cpp}
│   scene/  anim.{h,cpp}  M2  deformers + keyframe playback
├─ trax/ ✅                      → Application (CPU driver)
├─ bvh-test/ ✅                  → Application (gtest)
├─ arches-check/                 → Application, the ONLY project linking rtm
└─ trax-cuda/                    → Application (CUDA), M5
```

### 3.1 Known defects to fix before M1

Verified in the tree, not speculation:

1. **`trax/main.cpp` `get_f32` is broken.** It reads
   `return it == _values.find(key) ? fallback : ...` — comparing the iterator against a second
   lookup of the *same* key, which is always true. Every `--scale` value is silently ignored
   and the fallback is always used. `get_u32` and `get`, which use `_values.end()`, are
   correct. Fix and add an args unit test; this is exactly the class of bug that produces a
   whole experiment run at the wrong scale.
2. **No robust/compatibility mode switch.** `BVH_ROBUST_SLAB` defaults to `0` and is only a
   compile-time define with no test coverage of the enabled path. See §3.2.
3. `bvh-test` has no `args`, `rayset`, or `harness` coverage because those modules do not
   exist yet.

### 3.2 Two explicit modes: robust and Arches-compatible

*New, and it corrects a real conceptual error in the previous plan.* Matching Arches was
treated as equivalent to being correct. It is not. Arches' intersectors have properties we
verified bit-for-bit and that are genuinely undesirable:

- an exactly-grazing ray hits on a box's `min` plane and misses on its `max` plane — asymmetric
  purely because of `rtm::min`/`max` operand order;
- an edge-on or zero-area triangle is **accepted with `t = NaN`** (no determinant guard, and
  every `if (x < lo || x > hi)` rejection is false against NaN). Once `h.t` is NaN the
  closest-hit guard stops rejecting anything, so one degenerate triangle can poison a ray.

These are matched deliberately so step counts are comparable. They must not define correctness.

```
robust_mode   DEFAULT for all correctness work and all quality/perf experiments.
              - Ize 2013 slab padding      (rounding-induced false misses)
              - explicit axis-parallel guard (the 0 * inf = NaN case)
              - determinant guard + NaN rejection on triangles
arches_mode   Bit-exact rtm parity. Used ONLY by arches-check and by experiment
              rows explicitly labelled mode=arches.
```

Implemented as a `constexpr` policy on the intersectors (`core/mode.h`), not a global
`#define`, so both paths compile and both are tested in one binary.

**The three fixes are independent, and assuming otherwise is a trap I fell into.** Ize padding
does *not* fix the grazing asymmetry: when `d[a] == 0` and the origin lies on a slab plane the
far bound collapses to `-inf`, and `-inf × 1.00000024` is still `-inf`. There is no way to
recover `+inf` from `{-inf, NaN}` after the multiply, so the zero-direction axis has to be
decided *before* it — a ray parallel to a slab is either inside it for all `t` or outside it
for all `t`. Hence a separate `axis_parallel_guard`.

Every result row records its mode. A number produced under `arches_mode` may not be reported
as a correctness result.

---

## 4. Working arrangement

Two tracks, one repo.

| Track | Location | Branch | Who |
|---|---|---|---|
| Primary implementation | `C:\dev\bvh-lab\bvh-lab` | `main` | **you** |
| Reference implementation | `C:\dev\bvh-lab\reference` | `reference` | Claude |

You write the code. The `reference` worktree runs ahead as a working implementation you can
diff against when something is wrong, and as validation level 5 — a third independent answer
for any given scene. Neither branch is merged into the other by default; `reference` exists to
be *read*, not pulled.

**On pacing.** The schedule is expressed as milestones with gates (§5), deliberately without
day estimates. The earlier "one paper per day" framing was not achievable and made ordinary
slippage look like failure. The reference worktree runs ahead so there is always a finished
version to consult, but a milestone is done when its gate passes, not when a day ends.

Nothing gets pushed anywhere without you asking.

## 5. Milestones

Milestones, not days. Each has an **entry condition**, a **result artifact**, and a
**stop/go gate**. A gate is a question with a yes/no answer, not a vibe. If a gate fails,
the next milestone does not start — the failure is the finding.

Estimates are deliberately absent. The previous "one paper per day" schedule was not
achievable and made slippage look like failure rather than like normal work.

### Status

**Verified against the primary tree at commit `bcf6d45` plus the stabilization
changes described below.**

| | Milestone | State |
|---|---|---|
| M0 | Correctness and experiment harness | **partial.** Modes, oracle, serialized ray sets (6 distributions) and per-run CSV exist, and every artifact now defaults beneath `results/<run_id>/` (14.8). The **harness gate is NOT satisfied**: there is no matrix runner, and the ray-set cache key does not cover every input that changes the rays (see 14.7). |
| M1 | Validated CPU BVH2 baseline | **partial.** Four builders x six ray distributions exist. Every workload row is now validated over its **complete** ray set with the tie-aware oracle before it is timed or written, and a row that fails is omitted rather than published. But only **one scene** has been measured, the timing methodology of 14.5 is still outstanding, and heatmaps are not yet comparable (14.4). |
| M2 | Dynamic-scene baseline | **not started.** No refit/rotation/rebuild policy comparison, no animation driver. |
| M3 | Uncompressed wide BVH | **partial.** Greedy and DP collapse work. All six ray sets for a topology are now obtained and validated **before** any row for that topology is written, so a topology that fails leaves behind neither timing rows nor analytic depth rows. Missing: octant-fixed traversal ordering, the 14.5 timing methodology, collapse-phase timing distribution, frame-cost models, multiple scenes, comparable heatmaps. **Not complete merely because the collapse tests pass.** |
| M4+ | CWBVH and later | not started. |

---

### Current findings (preliminary, ONE scene, CPU only)

Teapot, 1024 tris, 512x512, robust mode, binned_sah base builder. This is a
pipeline demonstration, not a builder or layout conclusion.

- **BVH4 reduces node visits while holding box tests roughly constant**
  (11.18 -> 5.96 node steps/ray on incoherent rays; 21.4 -> 20.4 box tests).
- **BVH8 reduces node visits further but increases box tests** (4.29 steps,
  26.5 box tests) -- a wide node tests every slot unconditionally.
- **Under the current scalar slot-array traversal, BVH4 is faster and BVH8 is
  not** (43.7 / 38.0 / 40.6 MRays/s for bvh4 / bvh2 / bvh8).
- **DP produces fuller trees and lower node-cost SAH but does not
  automatically improve slot-aware SAH or wall clock.** DP width 8 has the best
  `sah_node_cost` (8.70) and among the worst `sah_slot_cost` (40.87).

Consistent with Meister 2022's finding that wide layouts need not beat binary,
reproduced here with the mechanism visible in `box_tests` rather than only in
wall-clock time.

**Explicitly NOT established:** anything about packed SIMD or GPU-wide
traversal, anything about a second scene, anything about hardware other than
this CPU.

---

### Invalidated results

**Every `overlap_d0` / `overlap_d3` / `overlap_d6` column in any `m3_wide.csv`
written before run `run_corrected` is INVALID and superseded.** Two defects:

1. `overlap_profile::nodes` was declared without a `{}` initializer while its
   sibling array had one, so counts -- and every mean derived from them -- read
   uninitialized memory. Fixed; regression tests in `bvh-test/overlap_test.cpp`
   fail deterministically without the fix.
2. The quantity was `sum(SA(child))/SA(parent)`: surface-area EXPANSION, not
   overlap. Disjoint children still inflate it. Renamed
   `mean_child_area_ratio`, with a real pairwise
   `SA(intersection(c_i,c_j))/SA(parent)` metric added alongside.

Superseded by `results/<run_id>/m3_depth_profile.csv`, which carries the full
per-depth profile instead of three sampled depths.

**Normalization, measured.** At depth 0 the unnormalized pair sum RISES with
width (0.215 / 0.440 / 0.690 for w2/w4/w8) purely because C(n,2) grows 1/6/28,
while `mean_pair_overlap` FALLS (0.215 / 0.073 / 0.025). They move in opposite
directions, so only the mean may be compared across widths.

---

### M0 — Correctness and experiment harness

Robust and Arches-compatible intersector modes (§3.2). Brute-force closest-hit **and any-hit**
oracle. Deterministic scenes, cameras, seeds. **Serialized ray sets** for the six workload
distributions. CSV/JSON experiment output carrying the full coordinate (§9). Structural
validators. Image and ray-set hashing. Fix the `get_f32` defect (§3.1).

*Mostly built.* What is missing is the part that makes everything downstream comparable:
`eval/rayset` and `eval/harness`. Ray sets currently exist only as "whatever the camera
generates", which is one distribution out of six, and every timing number so far is
primary-visibility only.

- **Artifact** — `results/m0_harness.csv`: one row per (scene × ray set × mode), brute force
  only, with hashes. Plus a `raysets/` directory of serialized ray batches.
- **Gate** — Can the harness re-run a previous row and reproduce its image hash bit-for-bit?
  Do closest-hit and any-hit oracles agree on occlusion for every ray set?

### M1 — Validated CPU BVH2 baseline

Sweep SAH (slow quality reference), binned SAH at 16 and 32 bins over centroid bounds, median
control. Ordered BVH2 traversal. SAH, EPO, depth, child overlap, leaf occupancy, traversal
counters. Refit. Differential validation against brute force and TinyBVH.

**The first meaningful artifact is one trustworthy BVH2 comparison across multiple scenes and
multiple ray types — not a large number of half-finished builders.** The current single-scene,
primary-rays-only table is not yet that.

- **Artifact** — `results/m1_bvh2.csv` across ≥4 real scenes × 4 builders × ≥4 ray sets, plus
  per-scene traversal heatmaps.
- **Gate** — (a) Every builder agrees with the oracle on every ray set, under §6.1's hit
  criterion. (b) Every row is **reproducible**: re-running reproduces the tree bit-for-bit and
  the metrics within a stated tolerance. (c) Every ranking change between scenes or ray
  distributions is **explainable** from the recorded metrics — not "stable". Rankings
  legitimately change by scene and ray distribution (§8), so demanding stability would be
  demanding the wrong thing; what must hold is that we can say *why* a ranking moved.

### M2 — Dynamic-scene baseline  ← moved much earlier

Previously deferred past the whole compression track. It is moved ahead of wide BVHs because
it exercises `bvh2` only, needs nothing from M3–M8, and is one of the project's stated goals.

Implement and compare: refit-only · refit + fixed rotation budget (Kensler / Kopta) · periodic
rebuild · quality-triggered rebuild · full rebuild every frame.

Keep three scenarios separate — they have different valid answers:
1. **deforming BLAS** (vertices move, topology fixed) — refit applies;
2. **rigid transform / TLAS** (instances move) — refit of the TLAS only;
3. **topology-changing** (primitive count changes) — refit does *not* apply; rebuild is forced.

- **Artifact** — `results/m2_dynamic.csv` plus per-frame plots of quality, update time, trace
  time, and total frame time, for each policy on each animation type (§10).
- **Gate** — Fit the break-even model (§10) on one set of frames, then test it on **held-out**
  frames/scenes. Validating a prediction on the same samples it was computed from is circular
  and tests nothing. Then run an actual **ray-count sweep** bracketing the predicted crossover
  and check the measured crossing falls inside the predicted interval. If it does not, either
  the model or the timing breakdown is wrong, and that is the finding.

### M3 — Uncompressed wide BVH

BVH2 → BVH4/BVH8 greedy collapse and the Ylitie dynamic-programming collapse. Wide traversal
with octant-based child ordering. Node fullness and child overlap by depth.

This milestone exists *before* compression because width alone does not guarantee faster
traversal. Meister 2022's conclusion is explicit: *"the best results are provided by binary
BVHs constructed by SBVH"* — wide layouts did not win in their setup. They also flag a
confound worth reproducing carefully: their wide and binary results use **different trace
kernels** (Lier et al. 2018 vs Aila & Laine 2009), so part of that gap is kernel, not layout.
The lab should reproduce the behaviour and separate those two causes before compression is
introduced.

- **Artifact** — `results/m3_wide.csv`: width 2/4/8 × greedy/DP × ray sets, with fullness and
  overlap-by-depth histograms.
- **Gate** — Is the wide-vs-binary difference explainable in terms of measured node steps and
  fullness, rather than only observed in wall-clock? Does DP collapse beat greedy on SAH cost?

### M4 — CWBVH

Encode/decode. Conservative origin/exponent quantisation. Octant-aware child ordering.
Compressed traversal stack. Byte-exact layout tests. Quantisation-expansion measurement.
Bytes per triangle and traffic proxies.

**Treat CPU CWBVH as functional validation, not a performance result.** Its motivation is GPU
divergence and memory traffic; a CPU timing number neither supports nor refutes it.

- **Artifact** — `results/m4_cwbvh.csv`: bytes/tri, quantisation expansion (SAH cost of the
  decoded tree vs the source tree), pixel-identical check against M3.
- **Gate** — Is decode conservative on every node (decoded bounds ⊇ source bounds)? Is the
  image identical to the uncompressed wide BVH? Is the node exactly the intended size?

### M5 — GPU traversal baseline  ← moved before H-PLOC and HEC

Separate kernels for BVH2, uncompressed wide, and CWBVH (P19). Evaluate coherent vs
incoherent, closest-hit vs any-hit, persistent threads, lane utilisation/divergence, node and
primitive traffic, stack traffic and spills.

Moved ahead of M6–M8 because **every remaining milestone's motivating claim is a GPU claim**.
Implementing H-PLOC and HEC first would mean measuring them on the one device where their
central argument does not apply.

- **Artifact** — `results/m5_gpu.csv`: MRays/s and lane utilisation per kernel × layout × ray
  set, coherent and incoherent.
- **Gate** — Does the GPU reproduce the *ordering* of layouts that the traffic proxies
  predicted on CPU? Where it does not, is the discrepancy explained (divergence, occupancy)?

### M6 — Fast construction

Morton codes, radix sort, Karras 2012 LBVH, H-PLOC. A minimal PLOC/PLOC++ is useful as a
conceptual and validation stepping stone. ATRBVH and PRBVH are **not** in the initial set.

**Reference availability, verified:**
- Arches has `//HPLOC, //TODO` at [`bvh.hpp:45`](../../dev/arches/include/rtm/bvh.hpp:45) — an
  enum comment, not an implementation.
- TinyBVH has binned SAH (`Build`), SBVH (`BuildHQ`), AVX/NEON builders, `BuildQuick`,
  `Optimize` (reinsertion, with stochastic/extreme variants), `Refit`, and the layouts listed
  in §7 — but **no H-PLOC and no HEC**.
- So H-PLOC comes from the paper plus any verified author reference implementation. Budget for
  that; it is the least-supported milestone in the plan.

A CPU "GPU-shaped" version (explicit chunking, lane loops, no recursion) is right for
correctness, but **H-PLOC's performance claim can only be evaluated on GPU**, which is why M5
comes first.

- **Artifact** — `results/m6_build.csv`: build time broken down by phase × builder, with the
  resulting tree's quality and trace cost.
- **Gate** — Does H-PLOC land on a build-time/quality trade-off curve consistent with LBVH and
  binned SAH on the same scenes? For the wide conversion: compare it against M3's collapse
  **given the same input binary tree and the same collapse heuristic**. Equivalence with *every*
  M3 collapse policy is not required and would not be meaningful — different heuristics are
  supposed to produce different trees.

### M7 — Fused collapsing

After H-PLOC, before HEC/HE2. Compare three paths:
1. H-PLOC → BVH2 → top-down wide conversion;
2. H-PLOC → BVH2 → DP wide conversion;
3. H-PLOC with fused bottom-up wide construction.

**Measure total construction *plus* traversal, never construction alone.** Fused collapsing can
improve build time while degrading traversal quality; a build-time-only comparison would
report that as a win.

- **Artifact** — `results/m7_fused.csv` with a total-frame-time column, not just build ms.
- **Gate** — At what ray count does each path win? (Same break-even structure as §10.)

### M8 — HEC/HE2 and memory-aware construction

Block-aware leaf feasibility · memory-based SAH (MSAH) · dynamic and greedy collapse · node
merging · leaf/primitive-block merging · duplicate-pointer suppression · 64 B and 128 B block
experiments.

**Define the formats structurally, not by class name.** "HECWBVH" and "HE2CWBVH" are C++
identifiers; neither one certifies paper fidelity. Read from the Arches source
(`include/rtm/hecwbvh.hpp`, `nvcwbvh.hpp`, `ftb.hpp`, `src/trax-kernel/include.hpp`):

| | legacy V1 `rtm::HECWBVH` | active `rtm::HE2CWBVH` |
|---|---|---|
| selected by | `USE_HECWBVH_V1 1` | `USE_HECWBVH_V1 0` — **the active setting** |
| `WIDTH` | 6 | 7 |
| node declaration | `union alignas(64) Node { struct {interior}; FTB ftb; }` | `struct alignas(64) Node` |
| leaf storage | **inline** — the `FTB` is an arm of the node union | **separate** `std::vector<FTB> ftbs` array |
| node size | `alignas(64)` sets ALIGNMENT, not size. The union contains an `FTB`, and `static_assert(sizeof(FTB) == 128)` holds (`ftb.hpp:25`), so a V1 node is **≥ 128 B**, not 64. | interior fields sum to 64 B: `base_child_index`(4) + `base_prim_index`(4) + `exts[3]`(12) + `i_mask`/`o_mask`(2) + `qaabb[7]`(42) |
| primitive block | same union | separate 128 B `FTB`, `MAX_TRIS = 8` |
| child pointer | `base_index` + per-slot offset | `base_child_index` / `base_prim_index` + per-slot offset |
| alias mechanism | `o_mask` | `o_mask` |

**Unverified:** neither HEC node has a `static_assert` on its size — only `FTB` does. The
sizes above are read off the field declarations, and the V1 ≥ 128 B figure follows from the
union containing an `FTB`. Add `static_assert`s on our own encoders rather than trusting these.

**Alias semantics, confirmed from source.** `decompress` at
[`nvcwbvh.hpp:225`](../../dev/arches/include/rtm/nvcwbvh.hpp:225) advances the child/prim
offset **only when the slot's `o_mask` bit is set**. A clear bit means the slot resolves to the
same physical index as the previous slot. That is declared aliasing, and it is why §6.1's
structural validator must not demand once-only reachability.

Treat **active HE2 as the baseline** for the modern paper-aligned representation and describe
V1 separately. Label every result with which one it used.

**Evidence requirement, and it is the strict part of this milestone.** Separate:

- **CUDA software traversal evidence** (`S-gpu`) — validates that a software GPU
  representation is correct and measures it on that GPU.
- **Arches / custom RT-core simulation evidence** (`S-sim`) — the only source in this lab that
  speaks to dedicated traversal hardware.

A CUDA kernel cannot, by itself, support a paper claim about dedicated RT hardware. **M8's gate
requires `S-sim` evidence for any hardware claim**; CUDA numbers may accompany it but may not
substitute for it.

- **Artifact** — `results/m8_hec.csv`: bytes/tri, node+block accesses per ray, unique sectors
  per ray, and (on GPU) DRAM bytes.
- **Gate** — Does MSAH change the collapse decisions measurably versus plain SAH? Is total
  memory traffic reduced on GPU, not merely predicted to be by the CPU proxy?

---

### Deferred, and why

| Item | Status |
|---|---|
| SBVH | Reference-only, for thin/overlapping geometry. Meister 2022 found SBVH binary BVHs best overall, so it is the honest quality ceiling — but spatial splits interact badly with block sharing (§11 C). |
| Treelet restructuring (TRBVH/ATRBVH) | Later. `Optimize` in TinyBVH covers the reinsertion family if a control is needed. |
| DOBB, UBVH/SOBB | Later; alternative bounding volumes are a separate axis. |
| TLAS/BLAS | M2 needs a minimal TLAS for the rigid-motion scenario; the full instancing story is later. |
| HLBVH, PRBVH, simulated annealing | Omitted initially. |
| Stackless traversal variants | Omitted initially. |
| Historical encodings | Omitted. |


## 6. Metrics

One row per experiment coordinate (§9). Never eyeball stdout.

### 6.1 Correctness

**Hit agreement.** Exact hit/miss agreement against brute force. For distance, an explicitly
justified tolerance rather than bitwise equality. For identity, **exact primitive id only when
the nearest hit is unique**; where several primitives tie at the nearest distance the oracle
supplies an accepted equivalence set and the test asserts membership in it.

This matters because tied nearest hits are common and legitimate: shared edges, duplicate or
coincident triangles, coplanar overlapping geometry, and any ray that grazes a seam. A test
demanding one specific id in those cases is asserting an arbitrary tie-break, not correctness,
and it will fail on real scenes for the wrong reason.

Targeted degenerate tests, each with its own case: shared edges · duplicate/overlapping
triangles · coplanar geometry · large coordinates · axis-parallel rays · signed zero in a
direction component · infinities · NaNs.

**Structural validation, alias-aware.** Do *not* require every physical node or block to be
reachable exactly once. Merged layouts point several logical child slots at one physical
allocation on purpose — Arches encodes exactly this with `o_mask`
([`nvcwbvh.hpp:234`](../../dev/arches/include/rtm/nvcwbvh.hpp:234)): a clear bit means the slot
reuses the previous physical index. The validator distinguishes four things:

| Concept | Meaning |
|---|---|
| logical child slot / edge | a slot in a node, of which there are exactly `WIDTH` |
| physical allocation | a node or primitive block actually stored |
| declared aliasing | slots the format *permits* to share one allocation |
| traversal-time duplicate suppression | the traversal collapsing repeated pointers so shared work happens once |

Required invariants: no cycles · no invalid or dangling references · no orphan physical
allocations · correct bounds and metadata · valid reference multiplicity · aliases only where
the format permits them · each aliased physical block processed according to the layout's
traversal rules.

**Encoding.** Conservative encode/decode (decoded bounds ⊇ source bounds) · image and ray-set
hashes · NaN/Inf detection on every hit record.

### 6.2 Tree quality

Textbook SAH, `c(N) = (1/SA(N))·[c_T·Σ SA(N_i) + c_I·Σ SA(N_l)·|N_l|]` — Meister 2022 eq. 2,
matching the implementation · EPO (Aila/Karras/Laine 2013) · sampled leaf-count variance ·
MSAH where applicable (M8) · child overlap · depth · node and leaf fullness · primitive
reference count (>1× for SBVH) · quantisation expansion.

### 6.3 Memory

Total bytes · bytes per triangle · node vs primitive-block accesses per ray · unique
sectors/cache lines per ray · stack traffic and spills · GPU L1/L2 hit rates · DRAM bytes ·
Arches node/leaf/cache/DRAM traffic.

### 6.4 Timing

Broken down, never as one number: primitive setup · Morton generation · sorting · topology
construction · collapse · merge · encode · refit · rotations · traversal · total frame time.

### 6.5 Evidence sources and confidence

*Revised.* The previous E1–E4 ladder implied simulator evidence outranks physical hardware,
and classified real CPU timings as "proxy". Both are wrong: a measured CPU time is a hardware
measurement of a CPU, and a simulator measures a *model*, whose value depends entirely on the
model's fidelity to the thing being claimed about.

Source is a **category**, not a rank:

| Category | What it is | What it characterises |
|---|---|---|
| **S-analytic** | SAH, EPO, node/leaf counts, bytes, quantisation expansion | the tree itself, exactly; nothing about time |
| **S-cpu** | measured CPU time, step counts, unique-cacheline counts | this CPU, this build, this scene |
| **S-gpu** | measured GPU time, lane utilisation, L1/L2, DRAM bytes | that GPU |
| **S-sim** | Arches cycle-level node/leaf/cache/DRAM traffic | the modelled hardware, bounded by simulator fidelity |

Two clarifications that were previously muddled:

- A **CPU-side model of memory behaviour** (our unique-64 B-lines-per-ray counter) is
  `S-analytic`, not `S-cpu`: nothing about the CPU's real cache is being measured. A CPU
  *timing* is `S-cpu` and is a genuine measurement.
- No category dominates. A claim about dedicated RT hardware needs `S-sim` (or real hardware);
  a claim about GPU divergence needs `S-gpu`; a claim about tree shape needs only
  `S-analytic`. Using a stronger-sounding source for the wrong question proves nothing.

**Confidence is recorded separately from source**, per row: repetitions · variance or
confidence interval · warm-up policy · clock/power-state controls (fixed clocks, or noted as
uncontrolled) · whether cross-validated against another implementation · known limitations of
that source. A single un-repeated timing with no variance is reportable, but it is reported as
what it is.

---

## 7. Validation

Five levels, cheapest first. All live in `bvh-test` or `arches-check`; none is a dependency of
`bvh`.

1. **Unit tests** (gtest, vendored from source) — one file per module, FoxRT style.
2. **Brute-force oracle** — closest-hit *and* any-hit, over serialized ray sets. Every builder
   × layout × codec must match on `hit.id` and `hit.t`, plus a matching image hash.
   Authoritative, slow, answers *"is this correct?"*
3. **TinyBVH — an independent implementation and quality reference.** *Downgraded from
   "quality oracle".* It is a well-engineered second opinion, not ground truth: a difference
   means one of the two is worse, and which one is a question, not a verdict. Useful because it
   answers *"is this in the right neighbourhood?"* — something brute force cannot detect at
   all. A builder that is perfectly correct and produces a 1.4× worse tree passes level 2
   silently.
4. **Arches — a compatibility and simulator reference.** *Not a tree-quality oracle.* Its
   binned SAH bins over primitive-AABB union rather than centroid bounds, its SAH cost uses a
   flat `c_node = c_prim = 64`, and its intersectors carry the degenerate-case behaviour in
   §3.2. What it *is* authoritative for: whether our layout and traversal are the same
   measurement the simulator would make.
5. **The `reference` worktree** — a *control* implementation, not an independent answer. It
   shares an author, a design and therefore a likely bug set with the primary, so agreement
   between the two is weak evidence and disagreement is the useful signal. Never cite it as
   corroboration for a correctness claim.

**Verified so far, with provenance.** Via `arches-check`, which links `rtm` and `bvh` in one
binary and compares bit patterns rather than epsilons. Produced in
`C:\devvh-lab
eference` @ `251263a`, Release x64 / MSVC v143, `arches_mode`,
`--scene=scenes/teapot.obj`, `rtm::BVH::BuildArgs{SAH, width 2, max_prims_collapse 1}`:
camera ray generation, ray/AABB, ray/triangle, whole-image closest-hit, and — for sweep SAH on
teapot — BVH node count, per-node flags, indices, counts and bounds, and per-ray node/prim step
counts, all bit-identical (2047 nodes; 57343 node / 6945 prim steps over 16384 rays).

Scope of that claim: **one worktree, one scene, one builder, one mode, primary rays.** It says
our traversal is the same measurement Arches would make under those conditions. It says nothing
about the primary worktree (which has no BVH2 at all), nor about any other builder or scene.

**Linking Arches is possible but awkward.** `arches-check` does it, and needed workarounds for
three headers that use types they do not include (`ray.hpp`/`vec2`, `texture.hpp`/`uvec2`,
`ftb.hpp`+`bvh.hpp`/`BitArray`), plus an `stb_image` implementation for `Texture2D`. Each is a
one-line upstream fix. The alternative — dumping via
[`_serialize`](../../dev/arches/include/rtm/bvh.hpp:976) and reading the dump — remains viable
and avoids the coupling entirely.

**Reference implementations worth reading rather than rediscovering:**

- [`tiny_bvh.h`](../../dev/tinybvh-main/tiny_bvh.h) — verified contents: `Build` (binned SAH),
  `BuildHQ` (SBVH), `BuildAVX`/`BuildNEON`, `BuildQuick`, `Optimize` (reinsertion, with
  stochastic and extreme variants), `Refit`; layouts `BVH`, `BVH_Double`, `BVH_GPU`,
  `BVH_SoA`, `BVH_Verbose`, `BVH4_GPU`, `BVH4_CPU`, `BVH8_CPU`, `BVH8_CWBVH`; `SAHCost` and
  `EPOCost` with `W_EPO = 0.71`. **No H-PLOC, no HEC.**
- [`kernels/traverse_cwbvh.cl`](../../dev/tinybvh-main/kernels/traverse_cwbvh.cl) — the CWBVH
  hot loop, and the intrinsic checklist for M5. **Not all of these are CUDA built-ins**, and
  tinybvh's own comments describe them loosely as "CUDA's native" — verify per intrinsic
  rather than inheriting that claim:

  | needed by CWBVH | CUDA status |
  |---|---|
  | `__popc` | genuine CUDA built-in |
  | `__activemask()` | genuine CUDA built-in (CUDA 9+) |
  | `__bfind` | **no CUDA built-in.** `bfind.u32` is a PTX instruction; in CUDA use inline PTX, or synthesise it as `31 - __clz(v)` for a non-zero `v` |
  | three-operand `fmin_fmin` / `fmax_fmax` (Aila–Laine VMIN/VMAX) | **no CUDA built-in.** These map to `vmin`/`vmax` PTX forms; expect inline PTX or a two-step `fminf` fallback and measure the difference |

  So the port is *easier* than tinybvh's OpenCL shim for two of the four and equivalent for
  the other two. Treat "which of these are built-ins" as something M5 confirms against the
  CUDA docs and the generated SASS, not something this plan asserts.
- [`bvh_article/bvh.cpp`](../../dev/bvh_article-main/bvh.cpp) — the cleanest binned-SAH
  `FindBestSplitPlane` / `Subdivide` / `Refit` in any of these codebases.
- `tinybvh-main/external/madmann91/bvh/v2/` — `sweep_sah_builder.h`, `binned_sah_builder.h`,
  `reinsertion_optimizer.h`, `mini_tree_builder.h`.
- `github.com/meistdan/hippie` — the framework behind Meister 2022, including its PLOC,
  PRBVH and SBVH implementations and its evaluation harness.

---

## 8. Claims this lab can and cannot support

*New section, and the most important guardrail in this document.*

### Can support

- Relative tree quality between builders **on the scenes measured**, at E1.
- Traversal step counts and their sensitivity to topology, at E1/E2 — and, because we match
  Arches bit-for-bit, these are directly comparable with Arches.
- CPU trace time for a given builder × layout × ray set, at E2.
- Build, refit and rotation timings and their break-even points, at E2.
- GPU throughput, divergence and traffic once M5 exists, at E3.
- Byte-exactness and conservativeness of an encoding, at E1.

### Cannot support

- **That a wide or compressed layout is faster, from CPU measurements.** Their motivation is
  GPU divergence and memory traffic. M4 is functional validation only.
- **That a builder is better in general.** Meister 2022 measured 12 scenes and still found
  scene-dependent ordering. Four scenes cannot generalise; say "on these scenes".
- **That SAH cost predicts trace time.** Meister 2022 Table 6 shows the time-per-unit-cost
  ratio varies *systematically by builder family* — top-down builders (SAH, SBVH) have lower
  values, bottom-up (PLOC) and optimisation (PRBVH) higher. Equal SAH cost from two different
  builder families does not mean equal trace time.
- **That unique-cache-lines-per-ray predicts real memory traffic.** E2 proxy only.
- **That anything is novel**, until a focused literature review says so.
- **That matching Arches means being correct** (§3.2).

### Confounds to control explicitly

- **Different trace kernels across layouts.** Meister 2022 compared wide BVHs with Lier et al.
  2018 and binary with Aila & Laine 2009. Some of their wide-vs-binary gap is kernel, not
  layout. When we compare layouts, hold the kernel family fixed or report both.
- **Reordering.** Ray reordering was worth ~7–8% in Meister 2022 and its benefit *shrinks* with
  branching factor. On or off must be part of the coordinate.
- **Leaf size.** Our BVH2 currently splits to one primitive per leaf (Arches parity). That is
  not a neutral choice and interacts with every collapse decision downstream.

---

## 9. Experiment coordinate

Every row identifies:

```
scene × frame × builder × layout × codec × maintenance × ray set × device × mode × evidence level
```

`mode` is robust/arches (§3.2); `evidence level` is E1–E4 (§6.5). A row missing any coordinate
is not a result.

**Ray sets** — primary visibility · short shadow/AO · reflection · first diffuse bounce ·
later diffuse/incoherent bounces · fully random incoherent. Serialized, SoA, reused across
every device so CPU and GPU numbers are comparable.

**Scenes** — enough real scenes to avoid scene-specific conclusions (Meister 2022 used 12;
four is a realistic floor here), plus synthetic scenes that isolate one property each: regular
tessellation · mixed primitive scales · thin/elongated geometry · high overlap · hair/foliage ·
orientation-sensitive geometry.

**Two evaluation modes, always reported separately:**
1. pre-generated identical ray sets, to isolate traversal;
2. end-to-end path tracing, to measure realistic frame time.

---

## 10. Dynamic-scene experiments

Animation types, at minimum:

- mild wave/twist deformation;
- articulated or rigid-subset motion;
- severe but topology-preserving deformation;
- chaotic/exploding motion;
- many independently moving instances (TLAS);
- an animation that returns to its initial pose (catches irreversible degradation).

Plot per frame: tree quality, update time, trace time, total frame time.

**Break-even.** The number of rays after which building a better tree pays for itself:

```
R_break-even = (T_rebuild - T_refit) / (t_ray_refit - t_ray_rebuild)
```

where `T_*` are one-off update costs and `t_ray_*` are per-ray trace costs. If the denominator
is ≤ 0 the refit tree is not actually worse and rebuilding never pays. Report `R` alongside the
frame's actual ray count — that ratio is the decision, and it is what makes "refit vs rebuild"
a measurement rather than an opinion.

---

## 11. Research directions

Preserved as open questions. **None is claimed novel** until a focused literature review says
so.

### A — Direct H-PLOC → HEC/HE2 construction

*Can H-PLOC directly construct an HEC/HE2-style block BVH using MSAH and fused wide-node
decisions — avoiding an intermediate BVH2 and a full re-encode — while preserving enough
traversal quality to improve total frame time?*

The merged-node paper explicitly suggests tightly integrating its greedy method with H-PLOC,
in the manner of fused collapsing. Variables: SAH vs MSAH · top-down vs fused collapse · merge
penalty · block size · wide arity · primitive-block fit policy · node ordering · node-only vs
node+leaf merging.

Depends on: M6, M7, M8.

### B — Dynamic compressed-BVH maintenance

*When is refit plus incremental compressed-block re-encoding better than a full H-PLOC rebuild
plus fused re-encode?*

Uses the four-operation decomposition in §2.2. Candidate strategy: track dirty `bvh2` nodes →
refit affected ancestors → rotate selected dirty subtrees → re-encode only dirty compressed
blocks → repair/split/dissolve invalidated merged blocks → rebuild when a quality or
repair-cost threshold is crossed.

Depends on: M2, M4, **M6, M7**, M8. The comparison baseline is a *full H-PLOC rebuild plus
fused compressed-layout generation*, so the H-PLOC (M6) and fusion (M7) milestones are hard
prerequisites, not optional context — without them there is nothing to compare against.

**Strongest candidate**: it sits at the intersection of the two things
this lab is already set up to measure well (dynamic scenes, compressed layouts), and the
break-even framework in §10 answers it directly.

### C — Spatial splits with shared primitive blocks

*Can several spatially split bounding boxes reference one primitive block, avoiding SBVH's full
primitive duplication?*

Ambitious follow-up, not an initial milestone. Note the tension: SBVH's reference duplication
is exactly what block sharing is trying to avoid, and block-level dedup may reintroduce the
traversal cost SBVH pays to remove.

---

## 12. Reading queue

**Now (M0–M2):**
Meister 2022 (methodology, and the SAH-vs-time caveat in Table 6) · Aila, Karras & Laine 2013
(quality metrics, EPO) · Wald 2007 (binned SAH) · Ize 2013 (robust traversal) · Kensler 2008
(rotations) · Kopta et al. 2012 (refit + rotations for animation).

**Next (M3–M5):**
Ylitie et al. 2017 (wide + compressed) · Aila & Laine 2009 and the 2012 Kepler/Fermi addendum ·
Wald/Benthin/Boulos 2008 or Ernst & Greiner 2008 (wide BVH origins) · Lier et al. 2018
(the wide-BVH trace kernel Meister used — needed to separate kernel from layout in M3).

**Later (M6–M8):**
Karras 2012 · Meister & Bittner 2018 (PLOC) · Benthin et al. 2022 (PLOC++) · H-PLOC 2024 ·
Barbier & Paulin 2025 (fused collapsing) · Haydel et al. 2026 (merged nodes / HEC) ·
Stich et al. 2009 (SBVH).

**Reference / deferred:**
Meister et al. 2021 survey (as an index, read by section) · Karras & Aila 2013 (treelets) ·
Bittner et al. 2013 (insertion) · Gottschalk 1996 + Woop 2014 + DOBB (alternative BVs) ·
Benthin et al. 2018 (compressed leaves) · Benthin et al. 2024 (DGF).

### Corpus inventory (`C:\data\HWRT-Papers`, verified by opening each)

| File | Paper |
|---|---|
| `A Survey on Bounding Volume Hierarchies...` | Meister et al. 2021 STAR |
| `Meister2022BVH` | Meister & Bittner 2022, GPU BVH performance comparison |
| `fastbuild` | **Wald 2007**, On fast Construction of SAH-based BVHs |
| `sbvh` | **Stich et al. 2009**, Spatial Splits in BVHs |
| `aila2009hpg_paper` | Aila & Laine 2009, GPU traversal efficiency |
| `aila2013hpg_paper` | **Aila, Karras & Laine 2013**, On Quality Metrics of BVHs |
| `paper-original` | **Ize 2013**, Robust BVH Ray Traversal (JCGT 2(2)) |
| `Tree_rotations_for_improving...` | Kensler 2008, tree rotations |
| `hwrt_rotations` | Kopta et al. 2012, BVH updates for animated scenes |
| `FastParallelConstruction` | **Karras 2012**, Maximizing Parallelism in the Construction of BVHs, Octrees and k-d Trees (HPG 2012) — the parallel binary radix tree |
| `TreeLetConstruction` | **Karras & Aila 2013**, Fast Parallel Construction of High-Quality BVHs (TRBVH treelet restructuring) |
| `ploc-tvcg` | **Meister & Bittner 2018**, PLOC (TVCG) |
| `HPLOC` | Benthin et al. 2024, H-PLOC |
| `CWBVH` | Ylitie et al. 2017 |
| `FusedCollapeforWBVH` | Barbier & Paulin 2025 |
| `HECWBVH` | Haydel et al. 2026, merged nodes |
| `DOBB` | DOBB-BVH |

The corpus covers the M0–M5 path completely, and M6–M7 substantially: Karras 2012, PLOC
(TVCG 2018), H-PLOC and Fused Collapsing are all present. The one acknowledged gap inside that
range is **PLOC++**, which sits between PLOC and H-PLOC; H-PLOC is readable without it, so M6
is not blocked, but the intermediate step is missing.

**Still missing, in priority order:**

1. **Recent (2025–2026) dynamic-geometry / animated-BVH work.** The newest dynamic reference in
   the corpus is Kopta 2012 — fourteen years old, predating every wide and compressed layout
   here. Given Research Direction B is the strongest candidate, this is the largest
   substantive hole.
2. **Lier et al. 2018**, "CPU-style SIMD ray traversal on GPUs" — the wide-BVH trace kernel
   Meister 2022 used. Without it their wide-vs-binary result cannot be separated from a kernel
   difference (§8).
3. **UBVH / SOBB** — oriented-bounding-volume follow-ups to DOBB.
4. Benthin et al. 2022 **PLOC++** (the PLOC → H-PLOC step); Aila, Laine & Karras 2012
   **Kepler/Fermi addendum** (revises several 2009 conclusions).

---

## 13. Corrections applied in this revision

| # | Previous claim | Correction |
|---|---|---|
| 1 | Roadmap as `BVH2 → CWBVH → H-PLOC → HECWBVH` | Mixes builder, layout and codec axes. Replaced with six orthogonal axes and milestones that advance one at a time. |
| 2 | Dynamic scenes deferred past compression | Moved to M2. It needs only `bvh2` and is a stated project goal. |
| 3 | GPU work last (Day 7) | Moved to M5, before H-PLOC and HEC, because every later milestone's motivating claim is a GPU claim. |
| 4 | "Compressed layouts cannot be refit in place" | Too strong. Decomposed into geometry refit / topology maintenance / node re-encode / block repair (§2.2). |
| 5 | TinyBVH as "the quality oracle" | Downgraded to an independent implementation and quality reference. |
| 6 | Arches as ground truth for quality | Split: compatibility and simulator reference, never a tree-quality oracle (§7). |
| 7 | "Metrics must rank median < binned < sweep" | Removed as a gate. Meister 2022 Table 6 shows time-per-unit-SAH varies by builder family; SAH, EPO and time may legitimately disagree (§8). |
| 8 | Cache lines/ray reported alongside real measurements | Evidence levels E1–E4 introduced; proxies must be labelled (§6.5). |
| 9 | Matching Arches treated as correctness | Two explicit modes; Arches' degenerate-case bugs must not define correctness (§3.2). |
| 10 | Day-based schedule with risk ratings | Replaced with milestones, entry conditions, artifacts and stop/go gates. |
| 11 | Silent assumption that H-PLOC/HEC references exist | Verified: Arches has an H-PLOC **TODO** only; TinyBVH has neither. H-PLOC comes from the paper. |
| 12 | HECWBVH treated as one thing | Verified: `USE_HECWBVH_V1 = 0`, active path is **HE2CWBVH — 7 slots, 64 B node, separate 128 B FTB blocks**. The union layout is the inactive V1. |


---

## 14. Instrumentation and artifact policy (added after the M3 review)

### 14.1 Representation naming

The collapsed structure is an array of 32-byte `bvh2_node` child records with a
variable child count. It is a **logical N-ary slot array** -- not a packed SIMD
BVH4/BVH8, not CWBVH, not a GPU layout. Result rows carry three separate fields
so the distinction cannot be lost:

    topology          bvh2 | bvh4 | bvh8
    physical_layout   slot32_aos
    traversal_kernel  scalar_distance_sorted | scalar_storage_lifo | scalar_octant_fixed

Timings and byte counts from `slot32_aos` must never be presented as evidence
about packed or GPU-wide traversal.

`bytes_per_tri` is renamed `node_bytes_per_tri` because it counts nodes only. A
`total_bvh_bytes_per_tri` including primitive indices and triangle payload is
still OUTSTANDING.

### 14.2 SAH naming

- `sah_node_cost` -- cost per logical node. The DP objective. Falls with width
  for structural reasons and is NOT comparable across widths.
- `sah_slot_cost` -- charges every child slot / box test. The meaningful
  cross-width analytic comparison.

Do not present a falling `sah_node_cost` as proof that BVH8 is better.

### 14.3 Overlap metrics

Two distinct quantities, never one column: `mean_child_area_ratio` (expansion)
and `mean_pair_overlap` (real geometric overlap, width-normalized). Per depth:
internal node count, invalid-parent count, pair count, mean child-area ratio,
mean/p95/max pair overlap, and the width-DEPENDENT sum retained only so the mean
can be recomputed. Full profile, every populated depth.

Semantics fixed in this revision, and pinned by tests in
`bvh-test/overlap_test.cpp`:

- **`mean_pair_overlap` is positive-INTERIOR overlap.** If any axis of the
  sibling intersection has extent `<= 0`, `box_intersection_area` returns zero.
  Children that merely touch therefore have exactly zero pair overlap: a shared
  face of zero thickness cannot force a ray into both children. Non-degenerate
  intersections keep `SA(intersection)/SA(parent)`.
- **p95 is nearest-rank**, index `ceil(0.95*N)-1`. The previous truncating form
  gave `floor(0.95*N)`, which for `N = 20` returned the maximum instead of the
  second largest.
- **A zero-area parent is counted, not dropped.** It still appears in
  `internal_nodes` and additionally in `invalid_parent_nodes`; it contributes no
  normalized sample, and the means divide by the valid contributions only, so
  structural counts never silently disappear from the profile.

### 14.4 Heatmaps -- OUTSTANDING

Required, not yet implemented:

- separate raw per-pixel counters (node steps, box tests, prim steps, tri
  tests, max stack) -- never summed into one number, never per-candidate
  normalized;
- a versioned lossless raw artifact (>= 32-bit counters) plus metadata (width,
  height, byte order, metric, topology, layout, kernel, ray-set hash, run id),
  so plots regenerate without re-tracing;
- one shared numeric scale per comparison group, true min/max/p50/p95/p99/p99.9
  recorded, robust-percentile scaling by default (p99), clipped pixels visibly
  marked;
- a perceptually ordered colormap (Viridis/Inferno) with a numeric legend;
- signed difference maps against a baseline, symmetric shared range;
- scene overlays, and hit-only / miss-only statistics;
- `tools/plot_heatmaps.py` doing the plotting from the raw artifact.

**No per-ray CPU std::chrono timing.** Too noisy and too intrusive to be a
traversal timing heatmap; real timing heatmaps are GPU/simulator evidence or an
external tool such as Nsight.

Aila et al. 2013 Fig. 2 visualizes ONE explicit counter (leaf nodes visited by
non-terminating primary rays), and used primary rays for the picture while the
actual analysis used diffuse rays. A clear primary-ray map does not replace
measurement over the full workload set.

### 14.5 Timing and confidence -- OUTSTANDING

Still outstanding in full. One naming defect was corrected in the meantime: the
builder-summary columns fed by `render_bvh2` are now `render_s` /
`render_mrays_s`, because they time a whole frame -- primary ray generation,
shading and image writes included -- and must not be read as pure traversal
timings. Per-ray-set traversal timings live in `m1_workload.csv` and
`m3_wide.csv` and are labelled `source_timing=S-cpu`. Renaming them did not make
them meet the methodology below:

- at least one genuinely UNMEASURED warm-up (the current `repeat()` counts its
  first pass as a sample);
- inner iterations sized so each sample lasts >= ~100 ms;
- median as principal result, with min/mean/stddev/CV retained;
- no I/O, validation, image generation or quality evaluation inside a timed
  region;
- collapse timed on a fresh source copy made OUTSIDE the timed interval;
- separate phase columns: primitive setup, build, collapse, reorder, refit,
  validation (excluded from cost), quality eval (excluded), trace;
- recorded thread count, CPU, compiler, build config, git commit, clock/power
  control status;
- differences below observed variance reported as indistinguishable.

### 14.6 Frame-cost models -- OUTSTANDING

Collapse does not belong to every frame.

    static reuse       first    = setup + build + collapse + trace
                       later    = trace
                       amort(F) = (setup + build + collapse)/F + trace
    rebuild per frame  frame    = setup + build + collapse + trace
    refit fixed        frame    = payload_update + refit + trace
    refit + recollapse frame    = payload_update + refit + collapse_or_reencode + trace

Break-even:

    frames_to_amortize = collapse_overhead / per_frame_trace_savings
    rays_to_amortize   = collapse_overhead_s / (bvh2_s_per_ray - wide_s_per_ray)

If the wide layout is not faster, report **no finite break-even** rather than a
negative number. Report F = 1, 2, 4, 8, 16, 32, 64. Keep measured and derived
values in separate columns.

### 14.7 Ray-set cache key -- OUTSTANDING

The filename currently encodes scene NAME, distribution, resolution, bounces
and seed. It does not cover: scene CONTENT hash, mesh scale / geometry hash,
full camera parameters, AO radius, incoherent count, generator version, or
numerical mode. Any of those can change the rays while the cache silently
serves stale ones. Store the full set in the file header and reject a cached
file whose metadata does not match exactly.

Builder identity must NOT be part of the key: identical geometry, camera and
generation parameters should deliberately reuse the same rays across builders,
which is what makes builder comparison valid.

### 14.8 Result identity

Every row of every CSV -- `m1_bvh2.csv`, `m1_workload.csv`, `m3_wide.csv`,
`m3_depth_profile.csv` -- carries `run_id`. Rows from different invocations are
never mixed in one file without it.

`run_id` defaults to `run_<YYYYmmdd_HHMMSS>_<microseconds>`; a seconds-resolution
stamp collides whenever two runs start within the same second. `--run_id`
overrides it.

**Every CSV and image defaults beneath `results/<run_id>/`**, so a run cannot
overwrite an earlier one. Explicit `--out`, `--csv`, `--workload_csv`,
`--wide_csv` and `--depth_csv` overrides are honoured verbatim. Earlier results
are retained under their own paths for provenance rather than deleted.

Each `m3_wide.csv` row records `depth_profile_csv` as the path this run actually
wrote, not a bare file name, so the profile belonging to the same run is
unambiguous. Its evidence source is split into `source_analytic=S-analytic`
(structure, SAH, byte counts) and `source_timing=S-cpu` (timings only), and each
row carries `rayset_hash`, `rays`, `hits` and the requested thread count.
