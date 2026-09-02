#!/usr/bin/env python3
"""E4: where in the tree the descriptor earns its keep.

Reads d1_depth.csv, which the probe writes with one row per (cell, parent depth)
holding pooled correct/incorrect/tie counts per score. Reports the win rate as a
function of the depth of the parent whose children were compared.

Why this matters beyond the Sponza-primary question: the collapse's DP makes its
retain/absorb decisions at particular depths, so a descriptor that only predicts
well in the deep tail may be predicting well where it cannot act.

These are POOLED counts, not per-ray clusters. The probe cannot emit per-ray,
per-depth tallies without a per-ray record whose size grows with tree depth, so
this file carries point estimates only and no confidence interval is computed or
implied. Treat a band with few pairs as unresolved.

Usage:  python experiments/direction_d/depth_breakdown.py results/<run_id>
                [--scores a,b,c] [--bands 0-3,4-7,...]
"""

import collections
import csv
import os
import sys

DEFAULT_SCORES = [
    "directional_mean_fill",
    "directional",
    "surface_density",
    "box_projected_ratio",
    "primitive_count",
]

DEFAULT_BANDS = [(0, 3), (4, 7), (8, 11), (12, 15), (16, 23), (24, 31), (32, 63)]


def band_label(lo, hi):
    return "%d-%d" % (lo, hi)


def load(run_dir, scene_filter):
    """(scene, ray_set) -> depth -> score -> [correct, incorrect, ties], pooled."""
    path = os.path.join(run_dir, "d1_depth.csv")
    if not os.path.exists(path):
        raise SystemExit("no d1_depth.csv in " + run_dir +
                         " (the run predates the E4 instrumentation)")

    cells = collections.defaultdict(lambda: collections.defaultdict(dict))
    pairs = collections.defaultdict(collections.Counter)
    scores = None
    max_depth = {}

    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if scene_filter and r["scene"] not in scene_filter:
                continue
            if scores is None:
                scores = sorted(k[:-len("_correct")] for k in r if k.endswith("_correct"))
            key = (r["scene"], r["ray_set"])
            d = int(r["parent_depth"])
            pairs[key][d] += int(r["discordant_pairs"])
            max_depth[key] = max(max_depth.get(key, 0), int(r["max_depth"]))
            for s in scores:
                slot = cells[key][d].setdefault(s, [0, 0, 0])
                slot[0] += int(r[s + "_correct"])
                slot[1] += int(r[s + "_incorrect"])
                slot[2] += int(r[s + "_ties"])
    return cells, pairs, scores, max_depth


def win(c, i):
    return (c / (c + i)) if (c + i) else float("nan")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    run_dir = sys.argv[1]
    scene_filter = None
    want = DEFAULT_SCORES
    bands = DEFAULT_BANDS
    for a in sys.argv[2:]:
        if a.startswith("--scenes="):
            scene_filter = set(a.split("=", 1)[1].split(","))
        elif a.startswith("--scores="):
            want = a.split("=", 1)[1].split(",")
        elif a.startswith("--bands="):
            bands = []
            for tok in a.split("=", 1)[1].split(","):
                lo, hi = tok.split("-")
                bands.append((int(lo), int(hi)))

    cells, pairs, scores, max_depth = load(run_dir, scene_filter)
    want = [s for s in want if s in scores]
    run_id = os.path.basename(run_dir.rstrip("/\\"))

    out_path = os.path.join(run_dir, "d1_depth_summary.csv")
    fields = (["run_id", "scene", "ray_distribution", "depth_band", "band_lo", "band_hi",
               "max_depth", "discordant_pairs", "share_of_pairs"]
              + [s + "_win_rate" for s in want]
              + [s + "_decided" for s in want])
    rows = []

    for key in sorted(cells):
        scene, ray_set = key
        total = sum(pairs[key].values())
        print()
        print("=== %s  %s   (%d pairs, tree depth %d)"
              % (scene.replace(".obj", ""), ray_set, total, max_depth[key]))
        print("| depth | pairs | share | " + " | ".join(want) + " |")
        print("|" + "---|" * (3 + len(want)))

        for lo, hi in bands:
            agg = {s: [0, 0, 0] for s in want}
            n = 0
            for d in range(lo, hi + 1):
                n += pairs[key].get(d, 0)
                for s in want:
                    slot = cells[key].get(d, {}).get(s)
                    if slot:
                        agg[s][0] += slot[0]
                        agg[s][1] += slot[1]
                        agg[s][2] += slot[2]
            if n == 0:
                continue

            share = n / total if total else 0.0
            print("| %s | %d | %.1f%% | %s |" % (
                band_label(lo, hi), n, 100.0 * share,
                " | ".join("%.4f" % win(agg[s][0], agg[s][1]) for s in want)))

            row = {
                "run_id": run_id, "scene": scene, "ray_distribution": ray_set,
                "depth_band": band_label(lo, hi), "band_lo": lo, "band_hi": hi,
                "max_depth": max_depth[key], "discordant_pairs": n,
                "share_of_pairs": "%.6f" % share,
            }
            for s in want:
                row[s + "_win_rate"] = "%.6f" % win(agg[s][0], agg[s][1])
                row[s + "_decided"] = agg[s][0] + agg[s][1]
            rows.append(row)

    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    print()
    print("wrote", out_path, "with", len(rows), "rows")

    # Where the pairs actually are. If they concentrate shallow, a descriptor
    # that only wins deep cannot matter much, and the reverse.
    print()
    print("=== median depth of a discordant pair ===")
    print("| scene | ray dist | p50 depth | p90 depth | max tree depth |")
    print("|" + "---|" * 5)
    for key in sorted(pairs):
        hist = pairs[key]
        total = sum(hist.values())
        if not total:
            continue
        acc, p50, p90 = 0, None, None
        for d in sorted(hist):
            acc += hist[d]
            if p50 is None and acc >= 0.50 * total:
                p50 = d
            if p90 is None and acc >= 0.90 * total:
                p90 = d
        print("| %s | %s | %s | %s | %d |" % (
            key[0].replace(".obj", ""), key[1], p50, p90, max_depth[key]))

    return 0


if __name__ == "__main__":
    sys.exit(main())
