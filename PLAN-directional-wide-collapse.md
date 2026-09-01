# Direction-Conditioned Wide-BVH Collapse — implementation and experiment plan

**Status:** revision 4. **Phase 0 approved and in progress; V7 excluded.** Phases 1-4
remain unapproved and unimplemented.

**Worktree state at authoring time**

| | |
|---|---|
| branch | `claude/direction-d-wp0-wp3-ca77f3` |
| HEAD | `fc3a28f528b55cbbb128b372a5601e52359ad12e` (*Add large-scene Direction D screening*) |
| merge base with `main` | `3e7c36160de50fc3ad47d56aa44b7b9fdf51979f` (= `main` HEAD) |
| status | clean except untracked `scenes/` assets, which are left alone |
| machine | AMD Ryzen 7 8745HX, 8C/16T, 31.8 GB RAM, 259 GB free disk |

Every statement below was checked against this branch. Where an earlier document
disagrees with the code, the code wins and the disagreement is called out.

Revision 3 changes are summarised in §14; revision 4 in §14.1.

---

## 1. Why the previous experiment answered the wrong question

The D1 experiment (`983bc54`, `27b7732`, `970f092`, `fc3a28f`) was internally
sound and its correctness machinery passed, but it gated on a question we do not
care about.

What it did: kept the binary SAH tree unchanged; probed each box-hit child
independently at each parent visit; asked whether the **projected-triangle**
descriptor `Q_raw` ranked the sibling containing a relevant primitive hit above
the sibling that did not; and treated that as the gate for a *binary split*
method.

What it never did: it never built a direction-aware tree, never compared two
trees, never measured node visits, child traffic, stack work or runtime, and
never touched the binary-to-wide collapse where the hypothesis actually lives.

D1 measured **`H`** — "does this subtree contain a useful hit?" — with a
*triangle*-projection descriptor, then used it to gate a decision about **`I`** —
"will a ray enter this box, and should these boxes share a wide node?" Those are
different quantities. Summed triangle projections overcount overlapping projected
geometry and cannot reconstruct arbitrary-direction projected area, whereas
**AABB** projected area is exact for any direction.

**What survives**, as a narrow ablation result:

> On four development scenes and four large local scenes, the projected-triangle
> emptiness descriptor provides little incremental information about which
> sibling subtree contains a relevant hit, over the non-directional control
> `triangle_surface_area_sum / SA(child_box)`. Pooled: 53.23% vs 53.42% (small
> set), 54.2% vs 54.0% (large set). Its calibration is real and monotone (AABB
> false-positive rate 90.22% → 53.89% across `Q_raw` quintiles) but largely
> reproduced by surface density alone.

**What must not be concluded.** That is not evidence that direction-aware wide
collapse fails. Different statistic, different purpose, unchanged tree. The
correct reading: do not build the collapse objective out of the triangle
emptiness term. Use exact projected **AABB** area for the visitation weight, and
keep the triangle term as an explicitly-labelled ablation.

---

## 2. Hypothesis

> **H1.** SAH's surface-area term is the isotropic average of the exact
> direction-conditioned projected AABB area. When a workload's ray directions are
> anisotropic, replacing that isotropic average with the workload's directional
> average — during **binary-to-N-wide collapse only** — produces a wide tree that
> performs strictly less actual traversal and child-data traffic on **held-out**
> rays, at no material cost in ordinary SAH quality, node count, depth or bytes.

Scope discipline, enforced throughout:

* the binary SAH build is **not** modified;
* traversal is **not** modified and remains exact — no probability ever skips an
  intersected node;
* nothing permanent is added to `bvh2_node`; directional weights are temporary
  conversion-time side data, discarded afterwards;
* child ordering and any-hit termination policy are held constant in this first
  experiment.

---

## 3. Mathematics, with assumptions and limits stated

### 3.1 Exact directional projected area

For an AABB with extents `(ex, ey, ez)`, the three face areas are

```
Fx = ey * ez        Fy = ex * ez        Fz = ex * ey
```

and for a unit direction `d`,

```
Aproj(B, d) = |d.x| * Fx + |d.y| * Fy + |d.z| * Fz
```

exact — the orthographic silhouette area along `d`. Since
`E[|d.x|] = E[|d.y|] = E[|d.z|] = 1/2` for a uniform direction on the sphere,

```
E_isotropic[ Aproj(B, d) ] = (Fx + Fy + Fz) / 2 = SA(B) / 4
```

This is the exact bridge between SAH and its direction-conditioned form.

### 3.2 Two line measures, not two levels of expressiveness

Two estimators are available. They encode different assumptions about **how many
root-entering rays each direction contributes**, and the experiment runs both.

Both use the **same** direction density: `w_k` is the empirical probability mass
of the training ray directions in direction bin `k` (`sum_k w_k = 1`), and `d_k`
is that bin's representative unit direction. Neither estimator reweights or
re-bins the training rays; they differ only in the denominator.

**(a) Ratio of means — the SAH-consistent line measure.**

```
q_rom(n) = ( sum_k w_k * Aproj(B_n, d_k) ) / ( sum_k w_k * Aproj(B_root, d_k) )
```

Answers: *of all workload rays that enter the root, what fraction enter `n`?*
The number of root-entering rays carrying direction `d` is itself proportional to
`Aproj(B_root, d)` under a uniform line measure, and this estimator inherits that
weighting. It is the direct anisotropic generalisation of the measure SAH already
assumes, and it reduces **exactly** to `SA(n)/SA(root)` when the direction density
is uniform.

**(b) Mean of ratios — a line measure reweighted by `1 / Aproj(B_root, d)`.**

```
q_mor(n) = sum_k w_k * ( Aproj(B_n, d_k) / Aproj(B_root, d_k) )
```

Answers: *averaged over the direction density itself, with every direction given
weight `w_k` regardless of how many of its rays actually enter the root, what is
the per-direction conditional entry probability?* Making each direction
contribute `w_k` of the total rather than `w_k * Aproj(B_root, d_k)` is exactly a
reweighting of the line measure by `1 / Aproj(B_root, d)`. Because
`E[X/Y] != E[X]/E[Y]`, it does not reduce to SAH under a uniform direction
density.

Revision 2 described (b) as assuming an "equal ray budget per direction class"
while simultaneously specifying `w_k` proportional to bin population. That was
contradictory. The statement above is the correct one: **`w_k` is the empirical
direction density in both estimators**, and the difference is solely the
denominator's line measure.

**What (b) actually costs.** Under isotropic directions and a **cubic** root box,
symmetry makes (b)'s coefficients proportional to `(1,1,1)` and it reproduces SAH
*decisions* (though not byte-identical weights). Under isotropic directions and an
**anisotropic** root box it does not: numerically, a root box with face areas
`(3.1, 0.7, 11.9)` and a uniform direction sample yields coefficient ratios
`(1.000, 1.179, 0.717)`. So mean-of-ratios **injects the root box's shape into
the node weights even when the rays carry no anisotropy at all.** That is a
property of the line-measure choice, and it is one of the things V5 measures.

Both are unconditional, **root-relative** weights — the probability that a ray
entering the root enters node `n` — not parent-relative conditionals. That is
what an additive collapse DP needs, and §4 shows the existing DP is already
written in exactly this form.

Finally: averaging child/parent ratios is **not** exactly equivalent to SAH under
any of these measures. The equivalence needs infinite non-terminating lines, a
uniform line measure, no origins inside the scene, and no `t_max` truncation.
Primary, AO and diffuse rays violate all four. This is the limitation Aila et al.
identify with Endpoint Overlap; the directional term does not repair it, and it
is a declared limitation.

### 3.3 The three-coefficient reduction — scope, and why it is unavoidable here

`Aproj` is linear in `(|d.x|, |d.y|, |d.z|)`, so for the **ratio-of-means**
numerator:

```
sum_k w_k * Aproj(B, d_k) = Wx * Fx + Wy * Fy + Wz * Fz

W = ( E[|d.x|], E[|d.y|], E[|d.z|] )   over the training ray directions
```

Define the **directional area** `Adir(B; W) = Wx*Fx + Wy*Fy + Wz*Fz`.

**The reduction is general, not special to ratio-of-means.** Revision 2 treated
the three-coefficient collapse as a property of ratio-of-means that mean-of-ratios
escapes. That was wrong. The following holds for the whole family:

> **Proposition.** Let the weight of node `n` be
> `weight(n) = sum_k a_k * Aproj(B_n, d_k)` for any coefficients `a_k` that
> depend on the direction bin but **not** on the node. Then
> `weight(n) = c_x*Fx_n + c_y*Fy_n + c_z*Fz_n` with
> `c_i = sum_k a_k * |d_k.i|`.
>
> *Proof.* Substitute `Aproj(B_n, d_k) = |d_k.x|Fx_n + |d_k.y|Fy_n + |d_k.z|Fz_n`
> and exchange the sums. The node enters only through `(Fx_n, Fy_n, Fz_n)`. ∎

Mean-of-ratios is exactly this case with `a_k = w_k / Aproj(B_root, d_k)`, because
the denominator depends only on the **fixed root box** and is therefore a
per-direction constant:

```
q_mor(n) = Fx_n*Cx + Fy_n*Cy + Fz_n*Cz,
   Ci = sum_k w_k * |d_k.i| / Aproj(B_root, d_k)
```

Verified numerically to machine precision (max relative error 2.2e-16 over 200
random node boxes, with an anisotropic root and 500 arbitrary weighted
directions).

**Consequences, and they are the central structural fact of this plan:**

* **Every direction-only, node-independent analytic visitation weight for an AABB
  lives in the same three-parameter family.** No choice of direction sampling,
  binning, normalisation or line measure escapes it. Increasing `K`, refining the
  angular resolution, or switching normalisation cannot add expressiveness — it
  can only move the coefficient vector `(c_x : c_y : c_z)`.
* Because the collapse DP is invariant to a global positive scale on all node
  weights (§4), the reachable model space is the **two-dimensional projective
  simplex** of non-negative coefficient ratios. V3 and V5 are two different paths
  through that same simplex, not two different levels of representational power.
* Therefore **V5 cannot measure what a three-moment representation discards**, and
  revision 2's framing of it was wrong. V5 is a **normalisation / line-measure
  ablation** (§6): it asks whether `1/Aproj(B_root, d)` reweighting is a better or
  worse prior than the SAH-consistent measure, and it quantifies the
  root-box-shape contamination shown in §3.2. That is a real and useful question,
  but it is not a question about angular information.

**What no member of this family can represent:**

* **Sign and orientation.** A ray set entirely along `+x` and one split 50/50
  between `+x` and `-x` give identical coefficients and identical trees, yet are
  completely different workloads. Sign is irrelevant to a box's silhouette area
  but decisive for front-to-back ordering and for which subtree tightens `t_max`
  first.
* **Joint structure** between direction components.
* **Per-subtree behaviour.** The coefficients are a single global statistic of
  rays entering the *root*. Rays that actually reach a deep node inside a corridor
  are a heavily filtered subset with a very different direction distribution. The
  model applies the same anisotropy at every node and every depth. It cannot
  express "this subtree sees mostly vertical rays while its sibling sees mostly
  horizontal ones."

**Escaping the family requires node-dependent information.** That is exactly the
deferred V6 (measured per-node visitation), and — given the Proposition — V6 is
the *only* route to more expressiveness, not merely one option among several.
This materially raises the value of V6 and lowers the prior that any analytic
direction-only weight can produce a large effect.

So the honest characterisation of the primary model is: **a global anisotropic
reweighting of the three axes.** It still changes decisions non-uniformly — a
slab-shaped node has a very different `Fx:Fy:Fz` ratio from a cubic one — but the
reachable space is small, fully described by `(lambda, W)`, and now known to be
the whole analytic design space rather than a convenient corner of it.

This sharpens the prediction: gains should appear where box anisotropy correlates
with ray anisotropy (Sponza/Bistro corridors under near-horizontal cameras) and be
near zero where either is isotropic (hairball).

### 3.4 The blended weights, with units

**V3 — ratio-of-means.** `Adir` has area units and equals `SA(B)/4` isotropically
(§3.1), so the factor 4 makes the two terms dimensionally comparable:

```
Aeff_rom(n; lambda, W) = (1 - lambda) * SA(B_n) + lambda * 4 * Adir(B_n; W)
```

* `lambda = 0` -> `SA(B_n)` exactly.
* `lambda = 1` -> pure directional area.
* `W` exactly `(0.5, 0.5, 0.5)` -> `SA(B_n)` for every lambda (see §3.5 for what
  "exactly" must mean numerically).

Expanded, `Aeff_rom(n) = ax*Fx + ay*Fy + az*Fz` with
`ax = 2(1-lambda) + 4*lambda*Wx`, and likewise for `y` and `z`.

**V5 — mean-of-ratios, dimensional form.** `q_mor(n)` is dimensionless (a
probability-like ratio), so it must be re-dimensionalised before it can be blended
with an area. Multiply by the fixed root surface area:

```
A_mor(n; C) = SA(B_root) * q_mor(n)
            = SA(B_root) * ( Fx_n*Cx + Fy_n*Cy + Fz_n*Cz )

Aeff_mor(n; lambda, C) = (1 - lambda) * SA(B_n) + lambda * A_mor(n; C)
```

`SA(B_root)` is chosen as the scale because it makes `A_mor` an area of the same
order as `SA(B_n)` and reproduces the familiar `SA(n)/SA(root)` normalisation when
the two measures agree. Any other positive constant would rescale the directional
term and therefore *re-parameterise lambda* rather than leave the result unchanged
(§3.5.1), so the choice is frozen and recorded rather than left implicit.

* `lambda = 0` -> `SA(B_n)` exactly, so the byte-identity gate (§3.5) holds for V5
  exactly as it does for V3.
* `lambda = 1` -> `SA(B_root) * q_mor(n)`.
* Isotropic directions do **not** give `SA(B_n)` for `lambda > 0`, because of the
  Jensen gap and the root-shape contamination of §3.2. This is a declared property
  of the mean-of-ratios line measure, and it is why the exact-isotropic
  byte-identity clause (W0.3a) applies to V3/V4 only.

Both weights are of the form `c_x*Fx + c_y*Fy + c_z*Fz` with non-negative
coefficients, as the Proposition of §3.3 requires.

### 3.5 Numerical identity: what is guaranteed and what is only tested

Two different identity claims, with different strengths. Conflating them was an
error in revision 1.

**Guaranteed by construction — λ = 0.** At `lambda = 0` and `mu = 0` the weight
array is filled by literally calling `aabb::surface_area()`. The DP then consumes
numerically identical inputs through identical arithmetic, so the emitted tree is
byte-identical to the current collapse. This is a structural guarantee, not a
numerical hope.

Computing `2*(Fx+Fy+Fz)` instead would **not** be safe: `surface_area()` computes
`(x*y + y*z + z*x) * 2.0f`, and f32 addition is not associative, so a different
association order can differ by one ULP, flip a `<` in the DP, and change the
topology.

**Implementation requirement, verified by test — exact isotropic fixture.** For
`W = (0.5, 0.5, 0.5)` *exactly*, `ax = ay = az = 2` for every lambda, so
`Aeff = 2*Fx + 2*Fy + 2*Fz`. Because multiplication by exactly `2.0f` is
error-free in binary floating point, this equals `2*(Fx + Fy + Fz)` — **provided
the three face-area terms are summed in the same association order as
`surface_area()`**, i.e. `(Fz + Fx) + Fy`, corresponding to
`(x*y + y*z) + z*x`. Under `/fp:precise` (set in `common.props`) the compiler
will not reassociate. This is therefore an **implementation requirement plus a
test**, not an automatic property, and the plan states it as such.

**Not byte-identical, and must not be asserted as such — sampled isotropic
rays.** A finite rayset drawn from a uniform sphere gives
`W ≈ (0.5, 0.5, 0.5)` with Monte-Carlo error of order `1/sqrt(N)`, never the
exact value. Such a rayset can only be checked with a **tolerance on tree
statistics** (node count, depth, SAH cost, changed-decision fraction), never with
`memcmp`. Both checks appear in the gates as separate clauses (§9, W0.3a/W0.3b).

This clause applies to **V3 and V4 only**. V5 has no exact isotropic reduction
(§3.4), so its isotropic behaviour is reported as a measured quantity — the
coefficient ratios `Cx:Cy:Cz` under a uniform direction sample, which quantify the
root-shape contamination — rather than gated on identity.

#### 3.5.1 Scaling `W` is a re-parameterisation of lambda, not an invariance

Revision 2 listed "positive scaling of `W` changes no decision" as a test. That is
false at intermediate lambda, and the corrected statements are:

* **Scaling the raw ray directions before normalisation changes nothing.**
  Directions are normalised per ray before the moments are accumulated, so
  multiplying every input direction by any positive scalar yields bit-identical
  `W` and a bit-identical tree. This remains a valid test.
* **Scaling `W` itself by `t > 0` is invariant only at `lambda = 1`**, where the
  weight becomes `4t * Adir` — a global positive scale, which the DP ignores (§4).
* **At intermediate lambda it is a lambda re-parameterisation.** Since

  ```
  (1-lambda)*SA + 4*lambda*t*Adir
      = (1 - lambda + lambda*t) * [ (1-lambda')*SA + 4*lambda'*Adir ],
      with lambda' = lambda*t / (1 - lambda + lambda*t)
  ```

  the weight vector for `(lambda, t*W)` is a positive multiple of the weight
  vector for `(lambda', W)`.

  **Correction recorded for Phase 2 (approved).** That identity is exact
  *algebraically*. It does **not** guarantee byte-identical output, because the
  two weight vectors are produced by different sequences of separately rounded f32
  operations and may differ by an ULP, which can flip a `<` in the DP. So:

  * if byte identity is wanted, **canonicalise the coefficient ratio** first —
    reduce `(c_x, c_y, c_z)` to a fixed normalisation (for example divide by
    `c_x + c_y + c_z`) and build both trees from the canonical triple, so the DP
    sees literally the same inputs;
  * otherwise state the test as **equivalence within tolerance**: coefficient
    ratios agree to a stated relative tolerance, node weights agree to a stated
    relative tolerance, and the resulting trees agree on node count, emitted depth
    and `sah_cost` within tolerance — with any decision differences counted and
    reported rather than asserted absent.

  The unconditional byte-identity guarantee remains **only** the `lambda = 0` case
  of §3.5, which holds by construction because those weights are produced by
  calling `surface_area()` itself.

### 3.6 Invalid cases

* `SA(B_root) <= 0`, `Adir(B_root; W) <= 0`, or non-finite: abort with an
  explicit error. Never silently fall back.
* A node with `SA = 0` already yields cost 0; `Adir` also yields 0. Behaviour
  unchanged, which is what the λ=0 gate requires.
* Assert `Wx, Wy, Wz >= 0` and `Wx + Wy + Wz >= 1` (since `|dx|+|dy|+|dz| >= 1`
  for a unit vector). Assert `Cx, Cy, Cz > 0` and finite.
* `Aproj(B_root, d) > 0` for every unit `d` whenever all three root extents are
  positive, so the V5 accumulation is well defined for any real scene; a ray whose
  `Aproj(B_root, d)` is non-positive or non-finite is nevertheless rejected and
  counted rather than trusted.
* Rays with zero or non-finite direction are excluded from `W` and `C` and
  counted; if more than 0.1% of training rays are excluded, the run fails.

### 3.7 The triangle-emptiness ablation, defined but demoted

The same linearity applies to the existing D1 descriptor `G = (p_yz, p_xz, p_xy)`
from `bvh/eval/directional_geometry.cpp`:

```
Udir(n; W) = Wx*p_yz(n) + Wy*p_xz(n) + Wz*p_xy(n)
Qdir(n)    = Udir(n; W) / Adir(B_n; W)
Edir(n)    = 1 - clamp(Qdir(n), 0, 1)

Aeff_empty(n) = Aeff(n; lambda, W) * (1 + mu * Edir(n))
```

Structurally identical DP, exactly `Aeff` at `mu = 0`. The brief's additive form
`L_empty = E_d[ P_entry * E_lower * subtree_cost ]` is implementable in the same
bottom-up pass but makes the objective self-referential in a way that breaks the
DP's optimality argument, so it is **deferred**. Primitive count alone is never
used as a subtree cost, and the emptiness term is never the primary visitation
weight.

---

## 4. How this maps onto the existing collapse DP

Verified against `bvh/build/collapse.cpp` on this branch.

`collapse_dp` computes, bottom-up, per node and slot budget `j`:

```
area       = node.bounds.surface_area()
leaf       : c_intersect * prim_cnt * area          (only if prim_cnt <= max_leaf_size)
internal   : c_traversal * area + cost_L[k] + cost_R[width - k]
distribute : cost_L[k] + cost_R[j - k]
```

so the minimised objective is

```
C = c_traversal * sum_{retained internal n} SA(B_n)
  + c_intersect * sum_{retained leaf l}     SA(B_l) * prim_count(l)
```

— exactly the intended `C_collapse` up to the constant `1/SA(B_root)`. The code
already works in the unconditional root-relative form, merely unnormalised. The
root normalisation is irrelevant to the argmin because every term scales with the
node weight.

**The entire intervention is: replace `area` with `Aeff(n)`.**

* `collapse_args` gains `const f32* node_weight{nullptr}` and
  `u32 node_weight_count{0}`;
* `collapse_dp` reads `const f32 area = w ? w[n] : node.bounds.surface_area();`
* weights are precomputed once into a `std::vector<f32>` sized `nodes.size()`;
* at λ=0 that vector is filled by calling `surface_area()` itself (§3.5).

`collapse_greedy` is untouched — it is not part of this experiment.

### 4.1 Four pre-existing issues, to be **measured** in Phase 0 before any claim

Revision 1 asserted that two of these *will* fail. That was arithmetic, not
measurement. Phase 0 measures each one; the fixes are worth applying regardless
because they are strictly better, but no failure is claimed until observed.

**(a) DP scratch allocation.** `dp_cache` holds
`decision decisions[max_collapse_width + 1]` — 32 entries × 8 B + 8 B =
**264 B per node**, independent of the requested width. San Miguel's binary tree
has 19,943,025 nodes (measured, `d1_summary.csv`), so the arithmetic gives
**≈5.27 GB of scratch** on a 31.8 GB machine, alongside a 638 MB node array, a
~10M-triangle mesh and a 1.3 GB OBJ load. *Phase 0 measures actual peak RSS for a
width-8 collapse on every scene.* The fix — size the table to `width + 1`
(≈80 B/node, ≈1.6 GB for San Miguel) — is applied regardless, since it is a pure
improvement with no behavioural change. A further fallback, a cost-only array
with decisions recomputed at emit time (≈800 MB), is designed but implemented
only if measurement shows it is needed.

**(b) Traversal stack capacity.** `bvh2_stack_size` is 128 on this branch (raised
from 64 for San Miguel's depth-62 binary tree). For a width-`W` tree each
internal pop replaces one entry with up to `W`, so the worst case is
`1 + (W-1) * D_wide` where `D_wide` is the **emitted** tree's depth. Revision 1
estimated `D_wide ≈ 62/3 ≈ 21` for San Miguel at width 8, giving 148 > 128 — but
`D_wide` is an output of the collapse and was never measured. *Phase 0 collapses
each scene at widths 4, 8 and 16 and records the actual emitted depth and the
actual observed `max_stack`.*

Regardless of what that measurement shows, the following is **mandatory and
unconditional**, because the current kernels have no bounds check on
`stack_size++` and an overflow would silently corrupt results rather than crash:

* `collapse_report` gains `emitted_max_depth` and
  `required_stack_depth = 1 + (width - 1) * emitted_max_depth`;
* a helper `bool stack_bound_ok(const collapse_report&)` is checked **for every
  generated tree, before any ray is traced**; a tree that fails produces no row;
* `bvh2_stack_size` is raised to a value that provably covers every tree the
  experiment generates, chosen from the Phase-0 measurements and then frozen;
* the existing `main.cpp` check `max_depth + 2 >= bvh2_stack_size` is a
  binary-tree bound and is replaced by the wide-correct one.

**(c) Missing counters.** Present in `trace_stats`: `node_steps`, `prim_steps`,
`box_tests`, `tri_tests`, `max_stack`. **Missing: child AABB hits / stack pushes,
and pushes later discarded.** In both kernels a push happens exactly when a child
box test passes, so one counter serves both; a second for entries popped and
discarded by `entry.t >= h.t` separates useful from wasted stack traffic. Add
`box_hit()` and `pruned_pop()` to `trace_stats` and no-ops to `null_stats`,
called from `traverse_bvh2.h`. Purely additive, and proven inert by W0.2.

**(d) EPO scale.** `quality::evaluate` with `compute_epo` is
`O(nodes × triangles)` — 19.9M × 9.97M for San Miguel. EPO is computed on the
**small scenes only**, as a control metric, and reported as "not computed" for
the large scenes. Declared limitation, not an omission.

---

## 5. Measurement vocabulary

Revision 1 called interior node visits "wide-node fetches". That conflated a
traversal event with a memory transaction and is corrected here. Each quantity
below is reported **separately**; none is presented as a measured hardware
memory transaction.

| reported quantity | source | definition | what it is a proxy for |
|---|---|---|---|
| `interior_visits` | `node_steps` | internal stack entries popped and expanded | traversal steps; loop-iteration count |
| `child_slots_examined` | `box_tests` | Σ `child_cnt` over interior visits | wide-node lane occupancy; SIMD/SIMT lane work |
| `logical_child_bytes` | derived | `child_slots_examined × sizeof(bvh2_node)` (32 B) | **logical** volume of child records the traversal had to look at, in *this* uncompressed AoS layout |
| `box_tests` | `box_tests` | ray/AABB tests performed | ALU work. Numerically equal to `child_slots_examined` in this kernel; reported separately because they diverge under a padded or compressed node layout |
| `box_hits` / `stack_pushes` | **new** | children whose box test passed and were pushed | downstream work and stack write traffic |
| `pruned_pops` | **new** | entries popped and discarded by `entry.t >= h.t` | wasted stack traffic |
| `leaf_visits` | `prim_steps` | leaf entries popped | leaf-block loads |
| `prim_tests` | `tri_tests` | ray/triangle tests | primitive ALU work; kernel-independent |
| `max_stack` | `max_stack` | peak stack occupancy | register/scratch pressure |
| `bvh_bytes` | derived | `nodes × 32` | representation size in *this* layout |
| `cpu_seconds` | timer | scalar kernel wall time | **weak** proxy; see below |

Explicit caveats, carried into both reports:

* `logical_child_bytes` is a **model**, not a cache-line or DRAM measurement. It
  counts bytes of child records the traversal logically consulted. It is not
  corrected for cache reuse, and it is specific to the 32-byte AoS slot layout —
  it is **not** a compressed-wide-node bandwidth number.
* No GPU claim is made from any of this. The closest-hit kernel performs an
  `O(W²)` insertion sort per node visit that a SIMD/SIMT kernel does not, so CPU
  timing systematically penalises wide nodes. Timing is secondary evidence at
  fixed width only, and is never used to compare widths. An actual GPU or Arches
  evaluation is a separate later phase with its own gate, out of scope here.

---

## 6. Tree variants

All variants share the same input binary SAH tree, the same primitive
permutation, `max_leaf_size = 1`, robust mode, the 32-byte AoS wide-node
representation, the unmodified traversal kernel, the same raysets, the same
cameras and scene scale.

| id | variant | collapse weight | status |
|---|---|---|---|
| **V0** | binary SAH tree | — | reference; the tree every variant collapses from |
| **V1** | conventional DP collapse | `SA(B_n)`, existing path | the wide baseline |
| **V2** | λ=0 through the new path | `Aeff(n; 0, W)` | must be byte-identical to V1 (W0.1) |
| **V3** | directional, ratio-of-means | `Aeff(n; λ, W)` over the λ grid | **the hypothesis** |
| **V4** | V3 + triangle-emptiness | `Aeff * (1 + mu*Edir)` | ablation: does the D1 term add anything? |
| **V5** | mean-of-ratios normalisation | `Aeff_mor(n; λ, C)` (§3.4) | **required line-measure ablation** |
| **V6** | measured per-node visitation | representative-ray counts | **deferred**; by §3.3 the *only* route to more expressiveness |
| ~~V7~~ | ~~coefficient sweep~~ | — | **excluded** (§6.1) |

Frozen λ grid: `{0, 0.125, 0.25, 0.5, 0.75, 1.0}`.
Frozen μ grid (V4, at the λ chosen by the frozen rule): `{0.25, 0.5, 1.0}`.
V5 runs over the same λ grid as V3.

**V5 is required, but its purpose has changed.** Revision 2 justified V5 as a test
of "what the three-moment reduction discards". By the Proposition of §3.3 that is
impossible: `q_mor` is *also* a three-coefficient linear functional of the node's
face areas, because its denominator depends only on the fixed root box. V5 and V3
are two paths through the same two-dimensional projective simplex. V5 therefore
tests, precisely:

* whether the `1 / Aproj(B_root, d)` line measure is a better or worse prior than
  the SAH-consistent one for real workloads;
* how much **root-box-shape contamination** it injects — measurable directly as
  the coefficient ratios `Cx:Cy:Cz` produced by an isotropic direction sample
  (§3.2), which should be `(1:1:1)` for an unbiased measure and demonstrably are
  not for a non-cubic root.

Because `C` is a closed form, V5 needs **no direction sample set and no
stratification**: `Ci = mean over training rays of |d.i| / Aproj(B_root, d)` is
accumulated exactly in the same single linear pass that produces `W`. Revision 2's
`K = 64` octahedral stratification and `direction_sample_hash` are therefore
**removed** — they were solving a problem that does not exist, and they were the
source of the `w_k` contradiction corrected in §3.2.

### 6.1 V7 is excluded

A coefficient sweep is **not** part of this experiment. The reasoning that
motivated it was overstated in two ways, both recorded here so the error is not
repeated:

* A 45-point barycentric grid is a **coarse sample of a continuous simplex**, not
  the simplex itself. Nothing measured on such a grid bounds the continuum.
* The Proposition of §3.3 covers **linear projected-area weights** with
  node-independent direction coefficients. It does not cover every conceivable
  direction-only analytic metric — a nonlinear function of the face areas, or one
  using any node property beyond `(Fx, Fy, Fz)`, falls outside it.

Therefore the claim "no direction-only analytic AABB weight can help" is **not
available to this experiment and must not be written**. If a coarse sweep is run
in a later phase, the strongest permissible negative statement is:

> None of the sampled global linear axis weights improved held-out work on these
> scenes.

It would be reported as an exploratory coarse sweep, never as an exhaustive
search or an upper bound over a family.

### Width

**Width 8 primary.** Against this repository's actual code: `bvh_ptr::child_cnt`
is 5 bits and `max_collapse_width` is 31, so widths 4, 8 and 16 are all
representable with no layout change, and the traversal kernel loops generically
over `child_cnt`.

* **Width 4** — useful sensitivity check: cheap, confirms the effect is not a
  width-8 artifact, and its `O(W²)` sort penalty is negligible.
* **Width 16** — run for **counters and bytes only**. Its timings are excluded
  from every conclusion. It is the most demanding case for the stack bound.

Both 8 and 16 depend on the §4.1(b) stack work.

---

## 7. Training and held-out evaluation

### 7.1 Frozen workload-family mixture

Pooled training must not let raw ray counts choose the mixture by accident — at
512×512 the families differ by an order of magnitude in ray count (e.g. 262,144
primary rays versus however many secondary rays survive a primary hit), so an
unweighted pool would be silently dominated by whichever family happens to be
largest on that scene.

Because `W` is a mean, family moments compose linearly. **Frozen family weights:**

```
alpha_primary    = 0.25
alpha_shadow_ao  = 0.25
alpha_diffuse_1  = 0.25
alpha_incoherent = 0.25

W_pooled = alpha_primary   * W_primary
         + alpha_shadow_ao * W_shadow_ao
         + alpha_diffuse_1 * W_diffuse_1
         + alpha_incoherent* W_incoherent

C_pooled = the same weighted sum applied componentwise to C_primary ... C_incoherent
```

Each `W_f` — and likewise each `C_f`, which is a mean over the same rays — is
computed from that family's **training** rays only, and the same frozen α blends
both componentwise. The equal
weighting is a deliberate, declared choice — no family dominates and the mixture
does not vary with scene geometry or hit rate. Per-family ray counts are recorded
alongside so the reader can see what the equal weighting overrode. A family whose
training set is empty or degenerate (fewer than 1000 usable rays) is dropped and
the remaining weights are renormalised, with the substitution recorded in the
row.

### 7.2 Training → evaluation matrix

Two specialisation policies. **Pooled is the headline**; per-family is the
optimistic upper bound. They are never averaged together.

| policy | trees per scene per width | `W` used to build | evaluated on |
|---|---|---|---|
| **P-pooled** | **1** | `W_pooled` from view A training rays (all four families, frozen α) | all four families × views **B, C** (held out) and × view **A** (same-view, reported separately) |
| **P-family** | **4** | `W_primary`, `W_shadow_ao`, `W_diffuse_1`, `W_incoherent`, each from view A | each tree evaluated **only** on its own family, × views B, C (held out) and × view A (separately) |

Explicit cell-by-cell matrix for one scene at one width:

| tree | built from | evaluated on (held out) | evaluated on (same view, reported separately) |
|---|---|---|---|
| V1 baseline | — (SAH) | A,B,C × {primary, shadow_ao, diffuse_1, incoherent} | — |
| V3-pooled | view A, `W_pooled` | B,C × all four families | A × all four families |
| V3-primary | view A, `W_primary` | B,C × primary | A × primary |
| V3-shadow_ao | view A, `W_shadow_ao` | B,C × shadow_ao | A × shadow_ao |
| V3-diffuse_1 | view A, `W_diffuse_1` | B,C × diffuse_1 | A × diffuse_1 |
| V3-incoherent | view A, `W_incoherent` | B,C × incoherent | A × incoherent |
| V4, V5 | view A, `W_pooled` | B,C × all four families | A × all four families |

Seeded families additionally train on seed **A** and are evaluated on seed **B**.

**Storage and runtime implications of workload-specific trees**, stated because
they are a real deployment cost and not a free win:

* **P-family multiplies resident hierarchy memory by the number of families.**
  At width 8 a San Miguel wide tree is on the order of a few hundred MB in this
  32-byte layout; four family-specific trees are four times that, which in a real
  renderer would usually be unacceptable for a single scene. P-family is
  therefore an *upper bound on achievable specialisation*, not a proposal.
* **Build cost scales the same way**: one DP pass per family per width per λ.
  Phase 0 measures collapse time; the matrix is sized so that P-family runs only
  at the single λ chosen by the frozen rule, not across the whole grid.
* **P-pooled costs exactly one tree**, identical in size to the SAH wide tree, and
  is the only policy that can support a deployment claim.
* If P-family wins substantially while P-pooled does not, the honest conclusion
  is *"the directional signal is real but only exploitable at a memory cost we
  have not justified"*, and the follow-up is per-node measured visitation (V6),
  not shipping four trees.

### 7.3 λ selection

Frozen rule, using **view A only**: the smallest λ in the grid whose training-view
`S(logical_child_bytes)` is within 1% (absolute, in ratio units) of the best λ on
that same training measurement. λ is never chosen from held-out results, and
held-out results are not inspected before the grid, workloads, cameras,
aggregation rules and gates are frozen.

### 7.4 Raysets

Workloads: `primary`, `shadow_ao`, `diffuse_1`, `incoherent`. `shadow_ao` remains
the repository's finite-AO distribution (`t_max = 0.1 × scene diagonal`) and is
never described as a direct-light shadow workload. Resolution 512×512; 262,144
incoherent rays. Each rayset is generated **once in memory** from the canonical
binary tree, hashed, and reused across every variant, width and λ. The on-disk
cache is not used: `rayset_filename()` still omits scene content, camera, scale,
AO radius, generator version and numerical mode.

Note carried forward from D1: scenes whose diagonal is under ~0.31 produce a
degenerate AO range because `t_min_default = 1/32` is absolute. All four large
scenes are far above that threshold; the check is re-run and recorded.

---

## 8. Scenes and cameras

Primary performance set — the four large local scenes already in the worktree.
Triangle, node and depth figures are **measured** values from
`results/d1_large_sampled_2026_08_31/d1_summary.csv` on this branch.

| scene | triangles | binary nodes | binary depth | family |
|---|---|---|---|---|
| `scenes/intel-sponza.obj` | 3,746,948 | 7,493,895 | 31 | architectural |
| `scenes/san-miguel.obj` | 9,971,513 | 19,943,025 | **62** | architectural |
| `scenes/hairball.obj` | 2,880,000 | 5,759,999 | 34 | organic, high depth complexity |
| `scenes/bistro.obj` | 2,829,873 | 5,659,745 | 35 | architectural |

`teapot.obj`, `cornell-box.obj`, `bunny.obj` and `utah_teapot.obj` are retained
**only** as unit-test and full-oracle fixtures.

**Provenance.** All four large files are local Blender-converted OBJs copied from
`C:/dev/arches/datasets`; `experiments/direction_d/scenes.csv` records SHA-256 but
`source_url` is `local:...` and `license` is `unknown`. They stay
`role=development`. **No result may be called a formal or publishable scene gate
until provenance is resolved.** The manifest is extended with camera parameters,
not replaced.

**San Miguel** is the designated stress case for both §4.1(a) and §4.1(b).

**Cameras.** `camera::frame_bounds` places the eye outside the bounding sphere
looking at the centroid, which for interior architectural scenes frames the
*outside* of a building and is not a representative workload. Cameras are
therefore frozen as explicit `camera(width, height, focal, position, target, up)`
tuples. Candidate views are screened on:

* primary hit fraction in `[0.55, 0.98]`;
* pairwise view-direction separation of at least 45°;
* median primary hit distance at least 5% of the scene diagonal.

Accepted tuples, hit coverage, median hit distance and a parameter hash go into
`experiments/wide_collapse/cameras.csv` and are frozen before any performance
number exists. Views are named A (training), B and C (held out).

---

## 9. Validation and gates

### 9.1 Uncertainty estimation — deterministic blocks, then scene summaries

Rays within a view are spatially correlated, and views and workloads within a
scene are not independent samples. The estimator is built accordingly.

**Blocks.** Every workload's rays are partitioned into **256 deterministic
contiguous ray-index blocks**. For `primary` at 512×512 these are exact 2-row
image bands (ray index = `j*512 + i`). For `shadow_ao` and `diffuse_1`, rays are
emitted in scanline order as a subsequence of pixels, so contiguous index blocks
remain spatially coherent bands. For `incoherent` they are fixed arbitrary
blocks. Block boundaries are recorded in the artifact. (A 2-D tiling would need
the originating pixel stored in `rayset`; that is an optional additive change,
deferred, because index bands already give spatial stratification without
touching the generator or its hashes.)

All metrics are compared as **ratios** `variant / baseline`, so a value below
`1.0` is an improvement and every threshold below is written as an upper bound.

**Level 1 — the per-cell ratio.** A *cell* is one fixed
`(scene, view, workload)` combination. Within a cell, the ratio for metric `M` is
a **ratio of sums over blocks**, not a mean of per-block ratios, so blocks
contribute in proportion to their ray count:

```
R_cell(M) = ( sum over blocks b of M_variant[b] ) / ( sum over blocks b of M_baseline[b] )
```

**Level 2 — the scene-level statistic and its CI.** Views and workloads are
**fixed factors, not random samples**: there are only 3 and 4 of them and they
were deliberately chosen, so resampling them would be dishonest. Blocks are the
only random unit. The scene statistic is the geometric mean of the cell ratios
over the fixed set of held-out cells:

```
S(M) = ( product over held-out cells c of R_c(M) ) ^ (1 / number_of_cells)
```

Its confidence interval uses a **common-block, cell-aligned bootstrap**, which
revision 2 left undefined:

1. block ids `0..255` are the same in every cell of a scene, so for bootstrap
   replicate `b` draw **one** resampled multiset of 256 block ids (with
   replacement, frozen seed, `B = 2000`);
2. apply **that same resampled block set to every held-out cell of the scene**,
   recomputing each `R_c(M)` as a ratio of sums restricted to the resampled
   blocks;
3. form `S^(b)(M)` from those cell ratios by the geometric mean above;
4. the scene CI is the 2.5th and 97.5th percentiles of `{ S^(b)(M) }`.

**Correction recorded for Phase 4 (approved).** Reusing one block draw across
cells is legitimate only when the blocks of different cells refer to **the same
source-pixel tiles**, i.e. when block `i` of every cell covers the same image
region. That does **not** hold in general here: `shadow_ao` and `diffuse_1` rays
are a *subsequence* of pixels (only those with a primary hit), so contiguous
ray-index block `i` covers a different image region than block `i` of `primary`;
and `incoherent` has no image at all.

The rule is therefore:

* **If** blocks are tied to identical source-pixel tiles across the cells being
  combined — which requires storing the originating pixel in the rayset and
  defining blocks by that pixel rather than by ray index — use one common block
  draw per replicate across those cells.
* **Otherwise** bootstrap blocks **independently within each fixed cell**, obtain
  a per-cell replicate distribution, and only then combine cell statistics into
  the scene geometric mean.

Phase 4 must decide which regime applies, per workload, and record it. The
default, absent the source-pixel change to `rayset`, is **independent within-cell
resampling**.

Gate W2 is evaluated against **this** scene-level CI. Cell-level CIs are also
reported, for diagnosis only.

**Level 3 — across scenes.** The geometric mean over the four scenes, always
published alongside **every individual scene value**. A four-scene bootstrap is
reported for completeness with an explicit note that `n = 4` makes it nearly
uninformative; it never carries a gate decision on its own.

### Gate W0 — correctness and baseline reproduction

Every clause must pass. A W0 failure is an implementation error, never a research
result.

| # | check |
|---|---|
| W0.1 | **λ=0 byte-identical.** V2 reproduces V1 exactly: `memcmp` of the node array, identical `prim_indices`, identical `collapse_report`, and identical counters on every ray of every workload, for every scene and width. |
| W0.2 | **Counter additions inert.** With `box_hit()` / `pruned_pop()` compiled in, every pre-existing counter is unchanged from `fc3a28f` on the four small scenes across all six ray distributions. |
| W0.3a | **Exact isotropic fixture.** With `W` set to the exact constant `(0.5, 0.5, 0.5)` — a fixture value, not derived from any rayset — every λ in the grid reproduces V1 **byte-identically**. This requires the face-area summation order of §3.5 and is the test that verifies it. |
| W0.3b | **Sampled isotropic rayset.** With `W` computed from a 1M-ray uniform-sphere rayset, every λ reproduces V1 to within tolerance on tree statistics: node count and emitted depth within 0.5%, `sah_cost` within 0.5%, changed-decision fraction below 2%. **No byte-identity is asserted or expected here.** |
| W0.4 | **Exact traversal equality.** For every variant, width, scene, workload and **every ray**: identical hit/miss, identical closest `t`, primitive id identical or inside the brute-force tie set, identical any-hit boolean, versus the V0 binary baseline. |
| W0.5 | **Oracle.** Small fixtures: 100% brute-force. Large scenes: the stratified sample of §9.2, **frozen by the Phase-0 benchmark** at 7168 rays per rayset (28 per block: 14 hit + 14 miss) for scenes at or below 5M triangles and 1536 rays per rayset (6 per block: 3 hit + 3 miss) for scenes above, on the V0 baseline. Combined with W0.4 this transfers to every variant. |
| W0.6 | **Structural.** Every wide tree: parents precede children; `child_cnt <= width`; child ranges contiguous and in bounds; every primitive slot appears exactly once; every node's box contains its children's; leaf `prim_cnt` respects the policy. |
| W0.7 | **Stack.** `required_stack_depth = 1 + (width-1) * emitted_max_depth <= bvh2_stack_size` checked for **every** generated tree before any ray is traced; observed `max_stack` never exceeds it. A tree failing this produces no row. |
| W0.8 | **Reconciliation and determinism.** All CSV counts and totals reconcile; one scene's full matrix is re-run and is byte-identical apart from timing columns; counters verified identical at `threads=1` and `threads=16` on one coordinate per scene. |
| W0.9 | **Suite.** Complete Release x64 build, 0 warnings, full `bvh-test` suite passing. |

### 9.2 Oracle sample — benchmarked, then frozen

Revision 1 pre-committed to 8192 rays per workload without knowing the throughput.
Instead:

**Phase 0 benchmarks** brute-force oracle throughput (triangle tests per second,
single-threaded and at 16 threads) on each large scene, using the parallelised
sampled oracle. From that measurement the sample size is chosen so the **whole
matrix's oracle cost fits a declared budget of 60 minutes**, and is then frozen
and recorded before any variant is evaluated.

**Stratification, frozen before use:**

* the sample is drawn from the same 256 blocks as §9.1, with an equal per-block
  quota, giving spatial coverage rather than a single contiguous region;
* within each block, the quota is split **50/50 between rays the V0 baseline
  reports as hits and as misses**, so miss rays — which exercise the deepest
  traversal and the most box tests, and which a hit-biased sample would
  under-represent — are guaranteed present. If a block has too few of one class,
  the shortfall is filled from the other and the substitution is recorded;
* selection within a class is by fixed stride from the block start — fully
  deterministic, no RNG;
* the realised counts (`sampled_rays`, `sampled_hits`, `sampled_misses`,
  `blocks_covered`, and any substitutions) are recorded per workload in the
  artifact so the coverage is auditable rather than asserted.

This replaces the previous 16-ray screen, which was indefensible.

### Gate W1 — the knob does something without wrecking the tree

Readiness, not performance. For at least **two adjacent nonzero λ**, on at least
**3 of 4** scenes, at width 8:

* **changed decisions**: symmetric difference between the set of binary nodes
  retained as wide-node children under λ and under λ=0 is between **5% and 60%**
  of eligible internal nodes;
* `sah_cost` ratio `<= 1.05` in geometric mean across scenes;
* `wide_node_count` ratio `<= 1.05`, `emitted_depth` ratio `<= 1.05`,
  `bvh_bytes` ratio `<= 1.05`;
* collapse remains deterministic and every W0 clause still passes.

All four are **upper bounds**: a variant that makes the tree smaller, shallower or
cheaper passes trivially, which is the intended behaviour.

### Gate W2 — actual held-out work improves, with no compensating regression

Designed so a reduction in one counter cannot pass while another gets worse. All
thresholds are written as **upper bounds on `variant / baseline`**, so
improvements of any size remain valid — revision 2 wrote the guardrails as
two-sided "within 2%" bands, which would have failed a large improvement.

**Primary improvement metric:** `logical_child_bytes per ray`. It already couples
interior visits with lane occupancy, so trading fewer visits for wider nodes does
not move it artificially.

**Co-primary, must not get worse:** `prim_tests per ray`.

**Guardrails, must not get worse:** `box_tests per ray`, `stack_pushes per ray`,
`max_stack`.

Pass requires **all** of:

| # | condition |
|---|---|
| 1 | `S(logical_child_bytes) <= 0.97` (a `>= 3%` improvement) in at least **3 of 4** scenes |
| 2 | `S(prim_tests) <= 1.005` in **every** scene, and the pooled held-out `S(prim_tests) <= 1.0` |
| 3 | `S(box_tests) <= 1.02`, `S(stack_pushes) <= 1.02`, `S(max_stack) <= 1.02` in **every** scene |
| 4 | the **scene-level** CI of §9.1 Level 2 for `logical_child_bytes` has upper bound `< 1.0` in at least **3 of 4** scenes |
| 5 | every held-out cell ratio `R_c(logical_child_bytes) <= 1.05` |
| 6 | conditions 1–5 hold for **two adjacent λ** values, not one isolated optimum |

`interior_visits/ray` is reported prominently but is **not** a pass metric,
precisely because on its own it is gameable.

CPU time is recorded and reported as secondary evidence only, at fixed width, and
never used to compare widths or to say anything about GPU behaviour.

### Gate W3 — generalisation rather than overfitting

1. `(1 - S_heldout) >= 0.5 * (1 - S_train)` for `logical_child_bytes`, i.e. the
   held-out improvement is at least half the training-view improvement;
2. every held-out cell ratio `R_c(logical_child_bytes) <= 1.05`;
3. λ was selected by the frozen training-only rule;
4. the **P-pooled** policy — not only P-family — satisfies W2.

### Any-hit

Termination and ordering are held constant: unmodified `occluded()`, stored-order
push, LIFO pop, first-hit termination. Honest caveat: collapse changes *which*
children exist and therefore their stored order, so for any-hit rays topology and
order cannot be fully separated here. What is held constant is the **ordering
rule**. Termination-aware ordering (Ogaki's `H_i / (I_i C_i)`) is a deferred
follow-up.

---

## 10. Implementation phases

Each phase ends at a **STOP** for review.

### Phase 0 — feasibility, instrumentation, camera freeze *(no experiment)*

Measure, do not assume:

* peak RSS and wall time for a width-4/8/16 DP collapse on every scene, before
  and after the `width+1` scratch fix;
* **emitted** wide-tree depth and `required_stack_depth` per scene per width;
  choose and freeze `bvh2_stack_size`;
* brute-force oracle throughput per scene, then freeze the stratified sample size
  (§9.2);
* OBJ load time, binary build time, node count, binary depth per scene.

Code: DP scratch sizing; `bvh2_stack_size` + `required_stack_depth` +
`stack_bound_ok`; `box_hit()` / `pruned_pop()` counters; parallelised stratified
sampled oracle; `experiments/wide_collapse/cameras.csv`.

**STOP.** Report measured feasibility, the frozen stack size, the frozen oracle
sample, and the frozen cameras.

### Phase 1 — direction statistics

`bvh/eval/direction_stats.{h,cpp}`:

```cpp
struct direction_weights
{
    // Ratio-of-means moments: E[|d.x|], E[|d.y|], E[|d.z|]   (V3, V4)
    double wx{0.5}, wy{0.5}, wz{0.5};
    // Mean-of-ratios coefficients: E[ |d.i| / Aproj(B_root, d) ]   (V5)
    double cx{0.0}, cy{0.0}, cz{0.0};
    u64    rays{0};
    u64    rejected{0};
    bool   valid{false};
};

// Both statistics are accumulated in ONE linear pass. C needs the root box, so
// this is called after the canonical binary tree exists.
direction_weights compute_direction_weights(const rayset& training, const aabb& root);

direction_weights isotropic_direction_weights();     // the exact (0.5,0.5,0.5) fixture
direction_weights blend_families(const direction_weights* per_family,
                                 const double* alpha, u32 count);   // frozen mixture
u64  hash(const direction_weights&);
bool is_isotropic(const direction_weights&, double tol);
```

Accumulation in `double`, in ray-index order, deterministic. Directions are
normalised per ray (rayset directions are unnormalised at generation — verified in
`rayset::generate`). Both `W` and `C` are means over the same rays, so the frozen
family mixture of §7.1 applies to each componentwise.

Tests:

* axis-aligned rayset → `W = (1,0,0)`;
* uniform-sphere rayset → `W = (0.5,0.5,0.5)` within Monte-Carlo error;
* **positive scaling of all input ray directions changes nothing** (bit-identical
  `W`, `C` and tree) — the valid form of the scaling test;
* **`C` closed form**: `Fx*Cx + Fy*Cy + Fz*Cz` reproduces a direct
  `sum_k w_k Aproj(B_n,d_k)/Aproj(B_root,d_k)` evaluation to within 1e-12 relative
  over random node boxes — the Proposition of §3.3 as an executable check;
* **root-shape contamination**: with a uniform-sphere rayset and a *cubic* root,
  `Cx:Cy:Cz` is `(1:1:1)` within Monte-Carlo error; with a strongly anisotropic
  root it is measurably not, and the measured ratios are recorded;
* zero/NaN directions rejected and counted; non-positive `Aproj(B_root,d)`
  rejected and counted;
* family blending reproduces the frozen α exactly for both `W` and `C`;
* hash stable and covers both `W` and `C`.

### Phase 2 — weighted collapse

`bvh/build/collapse_weights.{h,cpp}`:

```cpp
struct collapse_weight_args
{
    double            lambda{0.0};
    double            mu{0.0};
    direction_weights directions{};
    bool              mean_of_ratios{false};   // V5: use C and SA(root), not W
    double            root_surface_area{0.0};  // V5 dimensional scale, frozen (§3.4)
    // Explicit canonicalised coefficients, used by the tolerance/equivalence
    // tests of 3.5.1. Not a variant; V7 is excluded (6.1).
    bool              explicit_coefficients{false};
    double            cx{0.0}, cy{0.0}, cz{0.0};
};

std::vector<f32> compute_collapse_weights(const bvh2&, const mesh&,
                                          const collapse_weight_args&);
```

Modify `collapse.h/.cpp` per §4. `collapse_report` gains `emitted_max_depth`,
`required_stack_depth`, `scratch_bytes`, `changed_decisions`.

The emptiness ablation reuses `compute_directional_geometry` unchanged. For San
Miguel that is 19.9M × 40 B ≈ 800 MB of side data, computed only for `mu > 0`
variants and released immediately.

Tests:

* λ=0 byte-identical on all small scenes and hand fixtures, for V3, V4 and V5;
* exact isotropic fixture byte-identical at every λ, **V3/V4 only** (W0.3a);
* sampled isotropic within tolerance (W0.3b);
* `W = (1,0,0)` at λ=1 produces the analytically expected different tree on an
  anisotropic fixture;
* **λ re-parameterisation under `W` scaling** (§3.5.1): the tree built with
  `(λ, t*W)` is byte-identical to the tree built with
  `(λ' = λt/(1-λ+λt), W)` for several `t` and λ. The revision-2 test that scaling
  `W` leaves decisions unchanged at intermediate λ is **removed as false**;
* V5 at λ=1 with explicit coefficients equal to `C` reproduces the V5 tree, and
  V3 at λ=1 with explicit coefficients proportional to `W` reproduces the V3 tree
  — confirming both live in the single coefficient family of §3.3;
* every emitted tree passes structural validation and the stack bound;
* deterministic across repeated runs.

### Phase 3 — runner

`trax/wide_collapse.{h,cpp}`; `--wide_collapse` / `--wide_collapse_only` in
`main.cpp` (additive and gated, exactly as `--direction_d` is today, so ordinary
M1/M3 behaviour is untouched); register in `trax.vcxproj{,.filters}`.

Per scene: load mesh once; build one canonical binary SAH tree
(`binned_sah`, 32 bins, `max_leaf_size = 1`, robust); `apply_reorder`; `refit`;
keep an immutable copy of the node array that every variant collapses from;
generate each rayset once and hash it; oracle-validate the baseline; then per
variant compute weights, collapse, validate structurally, check the stack bound,
verify exact equality with the baseline on every ray, and measure. A variant
failing any check writes **no** row.

### Phase 4 — run, aggregate, report

`experiments/wide_collapse/aggregate_wide.py` — deterministic; the only RNG is
the frozen block bootstrap of §9.1.

---

## 11. Artifacts

`results/<run_id>/`:

| file | grain | est. size |
|---|---|---|
| `w1_variants.csv` | scene × width × variant × λ × μ × policy — tree structure, SAH cost, bytes, changed decisions, collapse ms, scratch bytes, emitted depth, required stack | < 200 KB |
| `w1_traversal.csv` | scene × width × variant × view × workload — every counter of §5, timings, validation status | < 1 MB |
| `w1_blocks.csv` | scene × width × variant × view × workload × block — the paired per-block metric used by the bootstrap | < 20 MB |
| `w1_directions.csv` | scene × view × family — `W`, `C`, `SA(B_root)`, ray counts, rejections, frozen α, hashes | < 20 KB |
| `w1_cameras.csv` | frozen camera parameters, hit coverage, median hit distance | < 10 KB |
| `w1_oracle.csv` | scene × workload — frozen sample size, realised hits/misses/blocks, substitutions | < 20 KB |

**Total ≈ 20 MB**, dominated by the block table, which exists because the
uncertainty estimator needs it. There is no per-ray CSV; the 168 MB per-ray
artifact D1 needed existed because its unit of inference was the ray, whereas
here it is the block and the tree.

---

## 12. Reporting

### 12.1 Plain-language — `experiments/wide_collapse/RESULTS-W1-SIMPLE.md`

Two pages, answering exactly: did directional collapse build different trees;
were they better on training rays; were they better on held-out rays; which
scenes and workloads improved or regressed; did memory and tree size change;
should this continue, be revised, or stop.

One scene-level table:

| scene | trees differ? | training view | held-out views | tree size | verdict |
|---|---|---|---|---|---|
| intel-sponza | yes / no | −x% child bytes | −y% child bytes | ±z% | **win / mixed / loss** |

plus at most five supporting numbers in prose. The D1 predictor result appears
only under a separate heading "Predictor diagnostics — not a tree result".

### 12.2 Technical appendix — `experiments/wide_collapse/RESULTS-W1.md`

Exact equations; exact commands; commit SHA and dirty flag; scene, camera,
rayset, `W` and `C` hashes; every frozen parameter (λ and μ grids,
family α, seeds, resolution, widths, block count, oracle sample, thresholds,
bootstrap seed); raw CSV paths with SHA-256; validation coverage including
realised oracle hit/miss/block counts; every reconciliation identity and its
result; per-variant per-width per-view per-workload counters for all of §5;
P-pooled versus P-family side by side with their storage cost; deviations;
limitations. Negative scenes and low-support workloads are published, never
omitted.

---

## 13. Estimated runtime and resources

Estimates, to be **replaced** by Phase-0 measurements.

| stage | per scene | notes |
|---|---|---|
| OBJ load | 1–8 min | San Miguel is a 1.3 GB OBJ; dominates |
| binary SAH build | 1–5 min | single-threaded BFS builder, 10M triangles |
| direction weights | < 1 s | one linear pass per family |
| collapse per variant | 5–30 s | `O(N × width²)` |
| rayset generation | 1–3 min | 3 views × 4 workloads, once |
| oracle sample | ≤ 60 min total | **budgeted**, sample size chosen from the Phase-0 benchmark |
| exact-equality check | 2–5 min | `O(rays)` per variant |
| traced measurement | 5–15 min | all variants × widths × views × workloads, multithreaded |

**Whole matrix: roughly 6–12 hours**, dominated by loading, binary builds and the
oracle. Naturally split per scene and per width, so it runs as resumable
background jobs.

**Memory**: peak is San Miguel — mesh + 638 MB binary nodes + DP scratch + a
destination node array. Phase 0 measures actual peak RSS; the `width+1` scratch
fix and, if needed, the cost-only fallback are the mitigations.

**Disk**: ≈20 MB of artifacts.

---

## 14. Change summary — revision 2 → revision 3

**The mathematical correction.** Mean-of-ratios is *also* exactly reducible to
three global coefficients, because its denominator is the **fixed root box** and
is therefore a per-direction constant:

```
q_mor(n) = Fx_n*Cx + Fy_n*Cy + Fz_n*Cz,
   Ci = sum_k w_k * |d_k.i| / Aproj(B_root, d_k)
```

Verified numerically to 2.2e-16 relative error. Revision 2's claim that V5 could
measure "what the three-moment reduction discards" was therefore wrong, and the
plan now generalises the point into an explicit **Proposition** (§3.3): *any*
node-independent per-direction reweighting collapses to three coefficients, so the
entire direction-only analytic design space is a two-dimensional projective
simplex. Escaping it requires node-dependent or measured visitation information —
the deferred V6, which is now the *only* route to more expressiveness rather than
one option among several.

Consequent changes:

1. **V5 dimensional weight defined** (§3.4):
   `A_mor(n; C) = SA(B_root) * q_mor(n)`, blended as
   `Aeff_mor = (1-λ)*SA(B_n) + λ*A_mor(n; C)`. λ=0 still gives `SA(B_n)` exactly,
   so V5 keeps the byte-identity gate. The scale `SA(B_root)` is frozen and
   recorded because any other constant re-parameterises λ rather than cancelling.
2. **`w_k` contradiction resolved** (§3.2). `w_k` is the **empirical direction
   density of the training rays** in *both* estimators; the difference is solely
   the denominator's line measure (uniform, versus reweighted by
   `1/Aproj(B_root,d)`). The "equal ray budget per direction class" phrasing and
   the contradictory population-weighted stratification are removed. Since `C` is
   a closed form, the `K = 64` octahedral sample and `direction_sample_hash` are
   deleted entirely — `C` is accumulated exactly in the same single pass as `W`.
3. **False scaling invariance removed** (§3.5.1). Scaling the raw ray directions
   before normalisation changes nothing (still a valid test). Scaling `W` is
   invariant **only at λ=1**; at intermediate λ it is a λ re-parameterisation, and
   the test is now the corresponding **equivalence**: `(λ, tW)` must give a tree
   byte-identical to `(λ' = λt/(1-λ+λt), W)`.
4. **Scene-level CI defined** (§9.1). A common-block, cell-aligned bootstrap: one
   resampled multiset of 256 block ids per replicate, applied to **every** held-out
   cell of the scene, cell ratios recomputed as ratios of sums over the resampled
   blocks, combined by geometric mean, percentiles taken over replicates. Views and
   workloads remain fixed factors. W2 is evaluated against this CI; revision 2
   referred to a "Level-1 CI" that did not exist at scene level.
5. **Guardrails written as upper bounds** (§9, W1/W2/W3). Every threshold is now
   `variant / baseline <= x`, so improvements of any magnitude remain valid.
   Revision 2's two-sided "within 2%" bands would have failed a large improvement.
6. **V5 reframed** (§6) as a required **normalisation / line-measure ablation**,
   with a concrete measurable target: the root-box-shape contamination it injects,
   which is `(1:1:1)` for a cubic root under isotropic rays but numerically
   `(1.000 : 1.179 : 0.717)` for an anisotropic one.
7. **V7 proposed** (§6) — subsequently **excluded**; see §14.1.

Everything else from revision 2 is unchanged: λ=0 byte identity by construction,
unmodified binary SAH construction, real alternative wide trees, held-out
evaluation, once-generated hashed raysets, exhaustive baseline-versus-variant
equality on every ray, the frozen family mixture, the training/evaluation matrix
and its storage cost, the separated measurement vocabulary, deterministic blocks,
the benchmarked-then-frozen oracle, the measured-not-asserted feasibility items,
the four external scenes, and the two-level reporting.

---

### 14.1 Revision 3 → revision 4

1. **V7 excluded** (§6.1). A 45-point barycentric grid samples a continuous
   simplex coarsely; it does not cover it. The Proposition of §3.3 covers *linear
   projected-area weights*, not every direction-only analytic metric. The claim
   "no direction-only analytic AABB weight can help" is withdrawn and must not
   appear anywhere. A later coarse sweep may report only that none of the
   *sampled global linear axis weights* helped.
2. **Common block resampling restricted** (§9.1). One shared block draw across
   cells is valid only when blocks are tied to identical source-pixel tiles;
   otherwise blocks are bootstrapped independently within each fixed cell before
   cell statistics are combined. Default, absent a source-pixel change to
   `rayset`, is independent within-cell resampling.
3. **λ-scaling equivalence weakened from byte identity to tolerance** (§3.5.1).
   The identity is algebraic; separately rounded f32 paths need not agree bitwise.
   Either canonicalise the coefficient ratio before building, or test equivalence
   within a stated tolerance and report any decision differences. Only λ=0 keeps
   an unconditional byte-identity guarantee.

Phase 0 is approved on this basis. Phases 1-4 remain unapproved.

---

## 15. Stop and review points

| # | after | deliverable | decision |
|---|---|---|---|
| **S0** | Phase 0 | measured feasibility (RSS, emitted depth, oracle throughput), frozen stack size, frozen oracle sample, frozen cameras | proceed only if San Miguel builds and collapses at widths 8 and 16 within measured budget |
| **S1** | Phases 1–2 | weighted collapse plus tests; **W0** on the small scenes | proceed only if λ=0 is byte-identical and both isotropic clauses pass |
| **S2** | Phase 3 | runner, full W0 on large scenes, **W1** | proceed only if the knob changes decisions without wrecking the tree |
| **S3** | Phase 4 | full matrix, **W2** and **W3**, both reports | the go/no-go on the hypothesis |

Honest outcomes at S3:

* **W2 and W3 pass** → direction-conditioned collapse works on this
  representation; next steps are compressed wide nodes, termination-aware
  ordering, and V6;
* **W1 passes, W2 fails** → the model changes trees but not work. Report the
  negative result; the revision is per-node measured visitation (V6);
* **W1 fails** → a single global 3-vector cannot move a SAH-optimal collapse.
  Report that, and treat it as evidence that the useful directional information
  is per-node rather than global;
* **V5 beats V3 materially** → the `1/Aproj(B_root,d)` line measure is the better
  prior. This says nothing about angular resolution — by §3.3 both are
  three-coefficient models — and the follow-up is to adopt that normalisation, not
  to add directions;
* a later **coarse coefficient sweep**, if run, can at most say that *none of the
  sampled global linear axis weights* helped on these scenes. It cannot bound the
  continuous simplex and cannot speak for nonlinear or non-face-area metrics
  (§6.1).

None of these is a failure of the experiment. Only a W0 failure is.

---

## 16. What this plan deliberately does not do

* does not modify the binary SAH builder;
* does not add any field to `bvh2_node`;
* does not make traversal probabilistic or approximate;
* does not reorder children or change any-hit termination;
* does not claim GPU performance from CPU timing, and does not present
  `logical_child_bytes` as a measured memory transaction count;
* does not use the projected-triangle emptiness term as the primary visitation
  weight;
* does not promote the local Blender-converted OBJ scenes to a formal scene gate;
* does not use the existing on-disk rayset cache;
* does not tune λ on held-out results;
* does not treat views, workloads or individual rays as independent samples.
