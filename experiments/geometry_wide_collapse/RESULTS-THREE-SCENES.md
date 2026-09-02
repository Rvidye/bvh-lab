# Geometry-Derived Direction D Across Three Scenes

**Scenes:** Intel Sponza, San Miguel, and Hairball  
**Tree:** one ordinary binary SAH tree per scene, collapsed to BVH8  
**Measured implementation:** clean commit `6097937d8a218b29e25ca9d1fbd46a6b6a0e279a`

## The question

Standard wide-BVH collapse uses the surface area of each node's bounding box to
decide which binary nodes should remain and which should be absorbed into a
BVH8 parent. Surface area knows the size of the box, but it does not know how
the triangles inside that box are oriented or how much projected empty space
the box contains.

Direction D adds that missing geometry information. For every binary subtree,
it accumulates the area of its triangles as seen along the X, Y, and Z axes. It
then compares those triangle projections with the corresponding projections of
the subtree's bounding box. A box that has a large projection but little
triangle surface in that projection receives an additional cost when the
collapse considers retaining it as an internal node.

This metric does not replace SAH. It is an extra build-time cost used only while
choosing the BVH8 topology. The final tree still contains ordinary AABBs and
uses the existing traversal. The temporary triangle statistics are discarded.
No camera rays, training view, ray directions, or measured traversal results
are used to construct any tree. Rays are introduced only after construction to
measure the finished trees.

## What was compared

Each scene uses the same binary SAH source tree for every variant. Three BVH8
results matter for the interpretation:

- **SAH baseline:** the repository's ordinary dynamic-programming collapse.
- **Direction D:** SAH plus the X/Y/Z projected-triangle loss.
- **Scalar control:** SAH plus a matched triangle-density loss that knows the
  total triangle area but not its direction.

The scalar control is necessary. If Direction D does no better than this
control, any benefit may come from knowing that a box contains little geometry,
not from knowing the orientation of that geometry.

The main comparison uses the preselected equal-strength setting, `mu = 1`. The
other tested weights are used only to check whether the behavior is stable. We
do not select a different weight after seeing which number looks best.

Every view contains the same number of primary rays, so the reported scene
average is simply the equal average of Views A, B, and C. Negative percentages
mean less traversal work than ordinary SAH; positive percentages mean more.

## Results in one table

| Scene | Node visits: View A | View B | View C | Three-view average | Box-test average | Result |
|---|---:|---:|---:|---:|---:|---|
| **Intel Sponza** | **-5.59%** | **-2.93%** | **-1.89%** | **-3.47%** | **-3.72%** | Consistent improvement |
| **San Miguel** | -0.22% | -2.24% | +0.74% | -0.58% | -0.62% | Too small and view-dependent |
| **Hairball** | -1.22% | -0.01% | +2.06% | +0.28% | -0.23% | No useful improvement |

Primitive work remains essentially unchanged on Intel Sponza and Hairball. On
San Miguel, View B trades its reduction in node and box work for a 1.28%
increase in primitive tests. No timing or GPU-performance claim is made.

## Intel Sponza: the metric helps

Intel Sponza is the only scene where Direction D reduces node visits and box
tests from every camera. The largest change is View A, where node visits fall by
5.59% and box tests by 5.98%. Views B and C also improve rather than merely
moving the work to another viewpoint.

The scalar control improves Sponza too, which says that ordinary triangle
density already contains useful information. At the same setting, however,
Direction D performs better than the scalar control on all three views. The
extra advantage is largest on View B. This is the clearest evidence in the
three-scene experiment that keeping the X/Y/Z components separate contributes
something beyond total triangle area.

The improvement is not free. Direction D changes 4.48% of the binary-node
retain/absorb decisions, increases the BVH8 node count by 0.09%, increases the
ordinary SAH score by 0.45%, and increases emitted depth from 14 to 15. Those
structural costs are small compared with the observed reduction in traversal
work for these three views.

The likely explanation is that Sponza contains many large, strongly aligned
architectural surfaces. Its AABBs and its triangle projections have a clear
relationship along the coordinate axes, so the three-component descriptor can
identify some internal boxes that are poor choices to retain during collapse.

| Ordinary SAH BVH8 | Direction D BVH8 |
|---|---|
| ![Intel Sponza ordinary SAH heatmap](../../results/geo_w8_sponza_2026_09_01/images/intel-sponza_view-A_w8-sah_traversal-heatmap.png) | ![Intel Sponza Direction D heatmap](../../results/geo_w8_sponza_2026_09_01/images/intel-sponza_view-A_w8-direction-D-mu1_traversal-heatmap.png) |

Both heatmaps use the same scale. The Direction D image is cooler across broad
regions, matching the measured reduction in traversal steps.

## San Miguel: the metric changes the tree but does not improve it reliably

San Miguel demonstrates the central weakness of the current metric. Direction
D changes 4.48% of collapse decisions and reduces the directional loss of the
finished tree by 10.13%. The implementation is therefore active and the
collapse successfully optimizes the number it was given. Actual traversal does
not follow that improvement: View B gets moderately better, View A is almost
unchanged, and View C gets slightly worse.

The scalar control is worse on average, but Direction D does not beat it on
every view. The distinction between directional orientation and simple density
therefore does not produce a clear practical benefit on this scene.

San Miguel contains many rooms, walls, openings, and layers of geometry behind
one another. The current descriptor adds the projected area of every triangle.
It does not calculate the visible union of those projections. Several surfaces
behind one another are counted several times, so a subtree can appear full even
when much of its box is empty or when its geometry is hidden behind another
surface. The descriptor also scores each retained node independently, while the
usefulness of a BVH8 grouping depends on how its children overlap and are
traversed together.

The BVH8 node count changes by only +0.012%, depth remains 49, and ordinary SAH
worsens by 0.51%. The structure remains similar in size, but its reorganized
nodes do not consistently save traversal.

| Ordinary SAH BVH8 | Direction D BVH8 |
|---|---|
| ![San Miguel ordinary SAH heatmap](san-miguel_view-A_w8-sah_traversal-heatmap.png) | ![San Miguel Direction D heatmap](san-miguel_view-A_w8-direction-D-mu1_traversal-heatmap.png) |

View A changes by only 0.22%, so these heatmaps are almost identical. Most rays
hit a nearby wall or floor and never exercise the deeper regions that were
reorganized.

## Hairball: the three-axis summary is too crude

Hairball does not benefit overall. View A improves slightly, View B is
unchanged, and View C regresses. Averaged across the three equal-size views,
node visits increase by 0.28% while box tests decrease by only 0.23%. This is a
redistribution of small amounts of work, not a better tree.

Direction D changes 4.75% of collapse decisions and reduces its own loss by
6.81%, but ordinary SAH worsens by 0.40% and emitted depth grows from 16 to 19.
The metric again has enough influence to create a different tree, but its lower
score does not correspond to lower traversal work.

Hairball is made from curved, overlapping strands with many orientations. Three
axis projections compress that complex orientation distribution into only
three totals. The totals also accumulate rapidly because many strands overlap
in projection. As a result, the descriptor loses the spatial and angular
structure that matters to traversal. The scalar control is also inconsistent,
which confirms that simple geometry density is not sufficient here either.

| Ordinary SAH BVH8 | Direction D BVH8 |
|---|---|
| ![Hairball ordinary SAH heatmap](../../results/geo_w8_hairball_2026_09_01/images/hairball_view-A_w8-sah_traversal-heatmap.png) | ![Hairball Direction D heatmap](../../results/geo_w8_hairball_2026_09_01/images/hairball_view-A_w8-direction-D-mu1_traversal-heatmap.png) |

The heatmaps are visually very similar, which agrees with the small View A
change.

## What the three scenes say about Direction D

The metric is neither inert nor broken. In every scene it changes roughly one
in twenty collapse decisions, and the resulting tree has a lower value of the
directional loss. Correctness is preserved. The important failure is that a
lower directional loss is not consistently connected to less traversal work.

The current evidence supports a narrow statement:

> Geometry-derived axis projections can improve wide collapse for Intel Sponza,
> but the same metric does not transfer reliably to San Miguel or Hairball.

Sponza suggests that directional triangle information can matter when geometry
is strongly aligned and the box-versus-triangle projection comparison remains
meaningful. San Miguel and Hairball show why the present three-number summary is
not a general replacement or universal supplement for SAH.

Two pieces of information are missing from the current loss:

1. **Projected union and depth.** It adds triangle projections even when they
   overlap, and it cannot tell which surface is in front.
2. **Relationships among candidate children.** Wide collapse chooses a group of
   children, but the loss judges each possible internal node separately. It does
   not measure whether the children overlap, are usually entered together, or
   form an efficient wide node.

The next useful version should therefore score the candidate BVH8 child group,
not merely add another property to each individual node. A geometry-only group
metric could examine projected overlap or occupancy among the candidate child
boxes along directions derived from their triangles. It should continue to be
measured against the scalar-density control so that any improvement can be
attributed to direction rather than geometry quantity alone.

Running more scenes with the current formula would characterize its failure
more thoroughly, but it would not repair the missing group information.

## Correctness and scope

- All 81 measured scene/view/variant rows passed traversal equality checks.
- The three scenes cover 2,359,296 direct ray comparisons with zero mismatches.
- At zero added weight, both geometry paths reproduce ordinary collapse
  byte-for-byte.
- The complete unit-test suite passes: 249 tests, 249 passed.
- Every reported run records clean commit `6097937` and zero rays used during
  construction.
- Materials are irrelevant to these measurements; the experiments use triangle
  positions, normals, AABBs, and traversal results.
- These are CPU traversal counters. They show tree work, not GPU speedup.
- Direction D changes wide-collapse topology. It does not rotate bounds like
  DOBB and does not change runtime child ordering.

## Raw measurements

- [Intel Sponza CSV](../../results/geo_w8_sponza_2026_09_01/intel_sponza_geometry_collapse.csv)
- [San Miguel clean CSV](../../results/geo_w8_san_miguel_clean_2026_09_02/san-miguel_geometry_collapse.csv)
- [Hairball CSV](../../results/geo_w8_hairball_2026_09_01/hairball_geometry_collapse.csv)

