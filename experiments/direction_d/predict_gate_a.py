#!/usr/bin/env python3
"""WP-A / WP-B: emit d1_prediction.csv and d1_contrasts.csv from probe data.

The sibling probe in bvh/eval/directional_probe.cpp already records, per ray,
how often each score ranked the box-hit sibling that actually contained the
relevant intersection above the one that did not. Nothing ever reported it.
This script does that analysis and nothing else -- no new instrumentation, no
re-run, no change to any tree.

Statistic
---------
    win_rate = correct / (correct + incorrect)

Ties are excluded from the denominator and reported separately. This differs
from the `accuracy` column in d1_summary.csv, which keeps ties in the
denominator; both are reported so the two cannot be confused.

Confidence intervals
--------------------
Clustered bootstrap resampling whole RAYS, not individual pairs. Candidate
events inside one ray share a direction, an origin and a traversal path, so they
are strongly dependent; a per-pair interval would be misleadingly narrow and
would make a null result look significant.

Rays that produced no discordant pair are real clusters that contribute
(0 correct, 0 incorrect). They are not written to the pairs CSV, so they are
reconstructed from `rays - rays_with_pairs` in d1_summary.csv and included in
the resample population.

How the resample is drawn
-------------------------
Drawing k rays one at a time is O(k) per resample, and k reaches ~5e5 per cell,
so 1000 resamples x 12 cells x 7 scores is not tractable in Python. Instead the
resample-count vector is drawn directly from its exact multinomial law by the
sequential conditional-binomial method (random.binomialvariate, exact for the
sizes here). This is the same classic nonparametric bootstrap, sampled by its
distribution rather than one draw at a time.

The collapse that makes this cheap: for a single score, a ray contributes only
the 2-vector (correct, incorrect), and few distinct 2-vectors occur, so the
multinomial has few categories. Collapsing on the score means each score's
interval is a MARGINAL interval -- correct for a per-score CI, but it carries no
information about the joint distribution of two scores.

Comparisons between two scores therefore get their own paired bootstrap in
d1_contrasts.csv, collapsing on the joint 4-vector (correct_a, incorrect_a,
correct_b, incorrect_b) so the two scores are resampled on the same rays. Those
paired intervals, not the point estimates, are what the gates are read from.

Usage:  python experiments/direction_d/predict_gate_a.py results/<run_id> [--scenes a.obj,b.obj]
"""

import collections
import csv
import gzip
import os
import random
import sys
import zlib

# The two controls Gate A cares about most. box_projected_ratio uses no triangle
# information at all, so it is the test of whether the descriptor adds anything
# over the bounding box.
DENSITY_CONTROLS = ["surface_density", "primitive_count"]
GEOMETRY_CONTROL = "box_projected_ratio"

# Gate A asks whether the directional descriptor predicts at all. Gate B asks
# whether the min-fill shape term beats the summed (mean-fill) one it replaces.
CONTRASTS = [
    ("directional", GEOMETRY_CONTROL),
    ("directional", "surface_density"),
    ("directional", "primitive_count"),
    ("directional_min_fill", "directional_mean_fill"),
    ("directional_min_fill", GEOMETRY_CONTROL),
    ("directional_mean_fill", GEOMETRY_CONTROL),
    ("directional_mean_fill", "directional"),
    ("directional_min_fill", "directional"),
]

BOOTSTRAP_RESAMPLES = 1000
BOOTSTRAP_SEED = 0xD1A70000D1A70000

CSV_FIELD_LIMIT = 1 << 24


def open_pairs(run_dir):
    plain = os.path.join(run_dir, "d1_directional_pairs.csv")
    if os.path.exists(plain):
        return open(plain, newline="")
    gz = plain + ".gz"
    if os.path.exists(gz):
        return gzip.open(gz, "rt", newline="")
    raise SystemExit("no d1_directional_pairs.csv[.gz] in " + run_dir)


def load_summary(run_dir):
    """(scene, ray_set) -> pooled support counts, summed over seeds."""
    path = os.path.join(run_dir, "d1_summary.csv")
    agg = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            key = (r["scene"], r["ray_set"])
            a = agg.setdefault(key, {
                "rays": 0, "rays_with_pairs": 0, "candidate_events": 0,
                "relevant_hit_events": 0, "false_positive_events": 0,
                "invalid_events": 0, "saturated_events": 0,
                "discordant_pairs": 0, "query_kind": r["query_kind"], "seeds": [],
            })
            for f_ in ("rays", "rays_with_pairs", "candidate_events",
                       "relevant_hit_events", "false_positive_events",
                       "invalid_events", "saturated_events", "discordant_pairs"):
                a[f_] += int(r[f_])
            a["seeds"].append(r["ray_seed"])
    return agg


def discover_scores(run_dir):
    with open_pairs(run_dir) as fh:
        header = fh.readline().strip().split(",")
    return [c[:-len("_correct")] for c in header if c.endswith("_correct")]


def seeded(*parts):
    """Reproducible per-(cell, statistic) stream, independent of iteration order."""
    tag = "|".join(str(p) for p in parts).encode("utf-8")
    return random.Random(BOOTSTRAP_SEED ^ zlib.crc32(tag))


class Cell:
    """Per-ray outcome tallies for one (scene, ray_distribution) cell."""

    def __init__(self, n_scores, n_contrasts):
        self.score_cats = [collections.Counter() for _ in range(n_scores)]
        self.contrast_cats = [collections.Counter() for _ in range(n_contrasts)]
        self.correct = [0] * n_scores
        self.incorrect = [0] * n_scores
        self.ties = [0] * n_scores
        self.rays_seen = 0


def load_cells(run_dir, scene_filter, scores, contrasts):
    csv.field_size_limit(CSV_FIELD_LIMIT)
    cells = {}

    with open_pairs(run_dir) as fh:
        rd = csv.reader(fh)
        header = next(rd)
        col = {name: i for i, name in enumerate(header)}
        i_scene, i_set = col["scene"], col["ray_set"]
        i_pairs = col["discordant_pairs"]
        i_correct = [col[s + "_correct"] for s in scores]
        i_ties = [col[s + "_ties"] for s in scores]
        pair_idx = [(scores.index(a), scores.index(b)) for a, b in contrasts]

        n_s, n_c = len(scores), len(contrasts)
        for row in rd:
            if scene_filter and row[i_scene] not in scene_filter:
                continue
            key = (row[i_scene], row[i_set])
            cell = cells.get(key)
            if cell is None:
                cell = cells[key] = Cell(n_s, n_c)
            cell.rays_seen += 1

            pairs = int(row[i_pairs])
            cor = [0] * n_s
            inc = [0] * n_s
            for j in range(n_s):
                c = int(row[i_correct[j]])
                t = int(row[i_ties[j]])
                cor[j] = c
                inc[j] = pairs - c - t
                cell.correct[j] += c
                cell.incorrect[j] += inc[j]
                cell.ties[j] += t
                if c or inc[j]:
                    cell.score_cats[j][(c, inc[j])] += 1

            for m, (ja, jb) in enumerate(pair_idx):
                k4 = (cor[ja], inc[ja], cor[jb], inc[jb])
                if any(k4):
                    cell.contrast_cats[m][k4] += 1

    return cells


def multinomial(rnd, k, masses):
    """Exact multinomial counts for the listed categories.

    `k` is the full cluster population, `masses` the multiplicities of the
    categories that can contribute. Whatever mass is not listed is the
    all-zero category; it is left as the remainder and never drawn, which is
    what keeps this O(len(masses)) rather than O(k).
    """
    out = []
    remaining_n = k
    remaining_mass = k
    binom = rnd.binomialvariate
    for m in masses:
        if remaining_n <= 0 or m <= 0:
            out.append(0)
            remaining_mass -= m
            continue
        if m >= remaining_mass:
            out.append(remaining_n)
            remaining_n = 0
            remaining_mass -= m
            continue
        c = binom(remaining_n, m / remaining_mass)
        out.append(c)
        remaining_n -= c
        remaining_mass -= m
    return out


def percentile_ci(values):
    if not values:
        return float("nan"), float("nan")
    v = sorted(values)
    lo = v[int(0.025 * len(v))]
    hi = v[min(len(v) - 1, int(0.975 * len(v)))]
    return lo, hi


def win_rate(correct, incorrect):
    d = correct + incorrect
    return (correct / d) if d else float("nan")


def bootstrap_score(cats, k, rnd):
    """Marginal percentile CI for one score's win rate, resampling whole rays."""
    if k <= 0 or not cats:
        return float("nan"), float("nan")
    items = sorted(cats.items())
    masses = [m for _, m in items]
    cor = [key[0] for key, _ in items]
    inc = [key[1] for key, _ in items]

    draws = []
    for _ in range(BOOTSTRAP_RESAMPLES):
        n = multinomial(rnd, k, masses)
        c = sum(a * b for a, b in zip(n, cor))
        i = sum(a * b for a, b in zip(n, inc))
        if c + i:
            draws.append(c / (c + i))
    return percentile_ci(draws)


def bootstrap_contrast(cats, k, rnd):
    """Paired percentile CI for win_rate(a) - win_rate(b) on the same rays."""
    if k <= 0 or not cats:
        return float("nan"), float("nan")
    items = sorted(cats.items())
    masses = [m for _, m in items]
    ca = [key[0] for key, _ in items]
    ia = [key[1] for key, _ in items]
    cb = [key[2] for key, _ in items]
    ib = [key[3] for key, _ in items]

    draws = []
    for _ in range(BOOTSTRAP_RESAMPLES):
        n = multinomial(rnd, k, masses)
        Ca = sum(x * y for x, y in zip(n, ca))
        Ia = sum(x * y for x, y in zip(n, ia))
        Cb = sum(x * y for x, y in zip(n, cb))
        Ib = sum(x * y for x, y in zip(n, ib))
        if (Ca + Ia) and (Cb + Ib):
            draws.append(Ca / (Ca + Ia) - Cb / (Cb + Ib))
    return percentile_ci(draws)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    run_dir = sys.argv[1]
    scene_filter = None
    for a in sys.argv[2:]:
        if a.startswith("--scenes="):
            scene_filter = set(a.split("=", 1)[1].split(","))

    scores = discover_scores(run_dir)
    contrasts = [(a, b) for a, b in CONTRASTS if a in scores and b in scores]
    print("scores found in the pairs CSV:", ", ".join(scores))
    print("contrasts:", ", ".join("%s-%s" % c for c in contrasts))
    print()

    summary = load_summary(run_dir)
    cells = load_cells(run_dir, scene_filter, scores, contrasts)

    run_id = os.path.basename(run_dir.rstrip("/\\"))
    fields = [
        "run_id", "scene", "ray_distribution", "query_kind", "score",
        "discordant_pairs", "correct", "incorrect", "ties",
        "win_rate", "win_rate_ci_lo", "win_rate_ci_hi",
        "accuracy_ties_in_denominator",
        "rays", "rays_with_pairs", "candidate_events",
        "relevant_hit_events", "false_positive_events",
        "invalid_events", "saturated_events",
        "bootstrap_resamples", "bootstrap_seed", "cluster_unit",
        "cluster_population", "seeds_pooled",
    ]
    contrast_fields = [
        "run_id", "scene", "ray_distribution", "score_a", "score_b",
        "win_rate_a", "win_rate_b", "delta", "delta_ci_lo", "delta_ci_hi",
        "a_beats_b", "bootstrap_resamples", "bootstrap_seed",
        "cluster_unit", "cluster_population", "pairing",
    ]

    rows = []
    contrast_rows = []

    for key in sorted(cells):
        scene, ray_set = key
        cell = cells[key]
        s = summary.get(key)
        if s is None:
            print("WARNING: no summary row for", key)
            continue
        if cell.rays_seen != s["rays_with_pairs"]:
            print("WARNING: %s %s has %d pair rows but summary says %d" % (
                scene, ray_set, cell.rays_seen, s["rays_with_pairs"]))

        # Every ray is a cluster, including the ones that produced no
        # discordant pair and so were never written to the pairs CSV.
        k = max(s["rays"], cell.rays_seen)

        for j, name in enumerate(scores):
            lo, hi = bootstrap_score(cell.score_cats[j], k, seeded(scene, ray_set, name))
            pairs = cell.correct[j] + cell.incorrect[j] + cell.ties[j]
            rows.append({
                "run_id": run_id,
                "scene": scene,
                "ray_distribution": ray_set,
                "query_kind": s["query_kind"],
                "score": name,
                "discordant_pairs": pairs,
                "correct": cell.correct[j],
                "incorrect": cell.incorrect[j],
                "ties": cell.ties[j],
                "win_rate": "%.6f" % win_rate(cell.correct[j], cell.incorrect[j]),
                "win_rate_ci_lo": "%.6f" % lo,
                "win_rate_ci_hi": "%.6f" % hi,
                "accuracy_ties_in_denominator":
                    "%.6f" % (cell.correct[j] / pairs if pairs else float("nan")),
                "rays": s["rays"],
                "rays_with_pairs": s["rays_with_pairs"],
                "candidate_events": s["candidate_events"],
                "relevant_hit_events": s["relevant_hit_events"],
                "false_positive_events": s["false_positive_events"],
                "invalid_events": s["invalid_events"],
                "saturated_events": s["saturated_events"],
                "bootstrap_resamples": BOOTSTRAP_RESAMPLES,
                "bootstrap_seed": "0x%X" % BOOTSTRAP_SEED,
                "cluster_unit": "ray",
                "cluster_population": k,
                "seeds_pooled": "+".join(sorted(set(s["seeds"]))),
            })

        for m, (a, b) in enumerate(contrasts):
            ja, jb = scores.index(a), scores.index(b)
            wa = win_rate(cell.correct[ja], cell.incorrect[ja])
            wb = win_rate(cell.correct[jb], cell.incorrect[jb])
            lo, hi = bootstrap_contrast(cell.contrast_cats[m], k,
                                        seeded(scene, ray_set, a, b))
            contrast_rows.append({
                "run_id": run_id,
                "scene": scene,
                "ray_distribution": ray_set,
                "score_a": a,
                "score_b": b,
                "win_rate_a": "%.6f" % wa,
                "win_rate_b": "%.6f" % wb,
                "delta": "%.6f" % (wa - wb),
                "delta_ci_lo": "%.6f" % lo,
                "delta_ci_hi": "%.6f" % hi,
                "a_beats_b": "yes" if lo > 0.0 else ("no" if hi < 0.0 else "inconclusive"),
                "bootstrap_resamples": BOOTSTRAP_RESAMPLES,
                "bootstrap_seed": "0x%X" % BOOTSTRAP_SEED,
                "cluster_unit": "ray",
                "cluster_population": k,
                "pairing": "paired",
            })

        di = scores.index("directional")
        print("  %-18s %-11s rays %-8d pairs %-9d dir %.4f" % (
            scene, ray_set, k,
            cell.correct[di] + cell.incorrect[di] + cell.ties[di],
            win_rate(cell.correct[di], cell.incorrect[di])))

    pred_path = os.path.join(run_dir, "d1_prediction.csv")
    with open(pred_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    contrast_path = os.path.join(run_dir, "d1_contrasts.csv")
    with open(contrast_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=contrast_fields)
        w.writeheader()
        w.writerows(contrast_rows)

    print()
    print("wrote", pred_path, "with", len(rows), "rows")
    print("wrote", contrast_path, "with", len(contrast_rows), "rows")

    by_cell = collections.defaultdict(dict)
    for r in rows:
        by_cell[(r["scene"], r["ray_distribution"])][r["score"]] = r
    by_contrast = collections.defaultdict(dict)
    for r in contrast_rows:
        by_contrast[(r["scene"], r["ray_distribution"])][(r["score_a"], r["score_b"])] = r

    # ---------------------------------------------------------------- Gate A
    print()
    print("=== Gate A: does the directional descriptor predict at all? ===")
    print("| scene | ray dist | directional | 95% CI | surf_density | prim_count "
          "| box_proj | > 0.5? | > density? | > box_proj (paired CI) |")
    print("|" + "---|" * 10)

    passes = 0
    for key in sorted(by_cell):
        c = by_cell[key]
        d = float(c["directional"]["win_rate"])
        lo = float(c["directional"]["win_rate_ci_lo"])
        sd = float(c["surface_density"]["win_rate"])
        pc = float(c["primitive_count"]["win_rate"])
        bp = float(c[GEOMETRY_CONTROL]["win_rate"])

        above_half = lo > 0.5
        ct = by_contrast[key]
        beats_density = all(
            ct[("directional", n)]["a_beats_b"] == "yes" for n in DENSITY_CONTROLS
            if ("directional", n) in ct)
        box_row = ct.get(("directional", GEOMETRY_CONTROL))
        beats_box = box_row["a_beats_b"] if box_row else "n/a"
        if above_half and beats_density and beats_box == "yes":
            passes += 1

        print("| %s | %s | %.4f | [%.4f, %.4f] | %.4f | %.4f | %.4f | %s | %s | %s |" % (
            key[0].replace(".obj", ""), key[1], d, lo,
            float(c["directional"]["win_rate_ci_hi"]), sd, pc, bp,
            "yes" if above_half else "NO",
            "yes" if beats_density else "NO", beats_box))

    print()
    print("cells passing all three Gate A criteria: %d of %d" % (passes, len(by_cell)))

    # ---------------------------------------------------------------- Gate B
    if "directional_min_fill" in scores and "directional_mean_fill" in scores:
        print()
        print("=== Gate B: does min-fill beat the summed (mean-fill) loss? ===")
        print("| scene | ray dist | min_fill | mean_fill | delta | 95% CI of delta | verdict |")
        print("|" + "---|" * 7)
        b_pass = b_fail = 0
        for key in sorted(by_contrast):
            r = by_contrast[key].get(("directional_min_fill", "directional_mean_fill"))
            if r is None:
                continue
            v = r["a_beats_b"]
            if v == "yes":
                b_pass += 1
            elif v == "no":
                b_fail += 1
            print("| %s | %s | %.4f | %.4f | %+.4f | [%+.4f, %+.4f] | %s |" % (
                key[0].replace(".obj", ""), key[1],
                float(r["win_rate_a"]), float(r["win_rate_b"]), float(r["delta"]),
                float(r["delta_ci_lo"]), float(r["delta_ci_hi"]), v))
        print()
        print("min_fill significantly better in %d cells, significantly worse in %d, "
              "inconclusive in %d" % (b_pass, b_fail, len(by_contrast) - b_pass - b_fail))

    return 0


if __name__ == "__main__":
    sys.exit(main())
