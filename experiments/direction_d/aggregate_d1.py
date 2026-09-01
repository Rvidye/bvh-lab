#!/usr/bin/env python3
"""Aggregate the Direction D artifacts into the tables used by RESULTS-D1.md.

Usage:  python experiments/direction_d/aggregate_d1.py results/<run_id>

Everything here is deterministic arithmetic over the committed CSVs. The only
randomness is the pooled scene bootstrap, whose seed is frozen in
experiments/direction_d/README.md section 7.

This script reports. It does not decide anything the README did not already fix.
"""

import csv
import gzip
import os
import random
import sys

SCORES = [
    "directional",
    "surface_density",
    "primitive_count",
    "box_surface_ratio",
    "box_projected_ratio",
]
CONTROLS = SCORES[1:]

VALID_BINS = [
    "q_00_10", "q_10_20", "q_20_30", "q_30_40", "q_40_50",
    "q_50_60", "q_60_70", "q_70_80", "q_80_90", "q_90_100",
]
QUINTILES = [
    ("Q1 [0.0,0.2)", ["q_00_10", "q_10_20"]),
    ("Q2 [0.2,0.4)", ["q_20_30", "q_30_40"]),
    ("Q3 [0.4,0.6)", ["q_40_50", "q_50_60"]),
    ("Q4 [0.6,0.8)", ["q_60_70", "q_70_80"]),
    ("Q5 [0.8,1.0]", ["q_80_90", "q_90_100"]),
]

# Frozen in the README.
SCENE_BOOTSTRAP_SEED = 0xD10000D1D10000D1
SCENE_BOOTSTRAP_SAMPLES = 2000

MIN_VALID_EVENTS = 10000
MIN_PAIRS = 1000


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load_manifest():
    here = os.path.dirname(os.path.abspath(__file__))
    rows = read_csv(os.path.join(here, "scenes.csv"))
    return {r["scene"]: r for r in rows}


def i(row, key):
    return int(row[key])


def f(row, key):
    return float(row[key])


def has(row, key):
    return row.get(key, "") != ""


def cell(row, key, fmt="%s"):
    """Accuracy-like columns are empty when the row has no discordant pair."""
    return (fmt % (100.0 * f(row, key))) if has(row, key) else "n/a"


def open_pairs(run_dir):
    plain = os.path.join(run_dir, "d1_directional_pairs.csv")
    if os.path.exists(plain):
        return open(plain, newline="")
    gz = plain + ".gz"
    if os.path.exists(gz):
        return gzip.open(gz, "rt", newline="")
    raise SystemExit("no d1_directional_pairs.csv or .csv.gz in " + run_dir)


def pct(x):
    return "%.2f" % (100.0 * x)


def ratio(num, den):
    return (num / den) if den else float("nan")


def coordinate_key(row):
    return (row["scene"], row["ray_set"], row["ray_seed"])


def check_reconciliation(summary, bins, pairs_handle):
    """The identities frozen in README section 12."""
    problems = []

    by_coord = {}
    for b in bins:
        by_coord.setdefault(coordinate_key(b), {})[b["ratio_bin"]] = b

    for s in summary:
        key = coordinate_key(s)
        bb = by_coord.get(key)
        if bb is None:
            problems.append("%s: no bin rows" % (key,))
            continue
        if len(bb) != 12:
            problems.append("%s: %d bin rows, expected 12" % (key, len(bb)))

        tot_c = sum(i(b, "candidate_count") for b in bb.values())
        tot_h = sum(i(b, "relevant_hit_count") for b in bb.values())
        tot_f = sum(i(b, "false_positive_count") for b in bb.values())

        if tot_c != i(s, "candidate_events"):
            problems.append("%s: bin candidate_count %d != summary %d" % (key, tot_c, i(s, "candidate_events")))
        if tot_h != i(s, "relevant_hit_events"):
            problems.append("%s: bin relevant_hit %d != summary %d" % (key, tot_h, i(s, "relevant_hit_events")))
        if tot_f != i(s, "false_positive_events"):
            problems.append("%s: bin false_positive %d != summary %d" % (key, tot_f, i(s, "false_positive_events")))
        if tot_h + tot_f != tot_c:
            problems.append("%s: hits + false positives != candidates" % (key,))
        if i(bb["invalid"], "candidate_count") != i(s, "invalid_events"):
            problems.append("%s: invalid bin != summary invalid_events" % (key,))
        if i(bb["raw_gt_1"], "candidate_count") != i(s, "saturated_events"):
            problems.append("%s: raw_gt_1 bin != summary saturated_events" % (key,))
        if i(s, "parent_visits") != i(s, "trace_node_steps"):
            problems.append("%s: parent_visits != trace_node_steps" % (key,))
        if i(s, "candidate_events") > i(s, "trace_box_tests"):
            problems.append("%s: candidate_events > trace_box_tests" % (key,))

        for b in bb.values():
            if i(b, "relevant_hit_count") + i(b, "false_positive_count") != i(b, "candidate_count"):
                problems.append("%s bin %s: hit + fp != candidates" % (key, b["ratio_bin"]))

        for name in SCORES:
            if i(s, name + "_correct") + i(s, name + "_ties") > i(s, "discordant_pairs"):
                problems.append("%s: %s correct + ties > pairs" % (key, name))

    # Pair rows, streamed: the file is the largest artifact.
    agg = {}
    rows_seen = 0
    with pairs_handle as fh:
        for r in csv.DictReader(fh):
            rows_seen += 1
            key = (r["scene"], r["ray_set"], r["ray_seed"])
            a = agg.setdefault(key, {"rows": 0, "pairs": 0, "cand": 0,
                                     "hash": r["rayset_hash"],
                                     **{n + "_c": 0 for n in SCORES},
                                     **{n + "_t": 0 for n in SCORES}})
            a["rows"] += 1
            a["pairs"] += int(r["discordant_pairs"])
            a["cand"] += int(r["candidate_events"])
            if int(r["discordant_pairs"]) <= 0:
                problems.append("%s ray %s: pair row with zero pairs" % (key, r["ray_index"]))
            for n in SCORES:
                a[n + "_c"] += int(r[n + "_correct"])
                a[n + "_t"] += int(r[n + "_ties"])

    for s in summary:
        key = coordinate_key(s)
        a = agg.get(key)
        if a is None:
            # A coordinate with no discordant pair correctly writes no pair row.
            if i(s, "rays_with_pairs") != 0 or i(s, "discordant_pairs") != 0:
                problems.append("%s: no pair rows but summary claims %d rays / %d pairs"
                                % (key, i(s, "rays_with_pairs"), i(s, "discordant_pairs")))
            for n in SCORES:
                if i(s, n + "_correct") or i(s, n + "_ties"):
                    problems.append("%s: no pair rows but %s has nonzero counts" % (key, n))
            continue
        if a["rows"] != i(s, "rays_with_pairs"):
            problems.append("%s: %d pair rows != rays_with_pairs %d" % (key, a["rows"], i(s, "rays_with_pairs")))
        if a["pairs"] != i(s, "discordant_pairs"):
            problems.append("%s: pair-row pairs %d != summary %d" % (key, a["pairs"], i(s, "discordant_pairs")))
        if a["cand"] != i(s, "candidate_events_in_pair_rows"):
            problems.append("%s: pair-row candidates %d != summary %d" % (key, a["cand"], i(s, "candidate_events_in_pair_rows")))
        for n in SCORES:
            if a[n + "_c"] != i(s, n + "_correct"):
                problems.append("%s: pair-row %s_correct mismatch" % (key, n))
            if a[n + "_t"] != i(s, n + "_ties"):
                problems.append("%s: pair-row %s_ties mismatch" % (key, n))

    # Coordinate identity: every row must carry the coordinates that make it
    # reproducible, and the three files must agree on them.
    for s in summary:
        key = coordinate_key(s)
        for col in ("run_id", "git_commit", "scene_sha256", "rayset_hash", "query_kind",
                    "width", "height", "ray_seed_value", "bootstrap_seed", "builder",
                    "bins", "max_leaf_size", "layout", "mode", "threads", "validation"):
            if not s.get(col):
                problems.append("%s: missing coordinate column %s" % (key, col))
        bb = by_coord.get(key) or {}
        for b in bb.values():
            if b["rayset_hash"] != s["rayset_hash"]:
                problems.append("%s: bins rayset_hash != summary" % (key,))
                break
            if b["scene_sha256"] != s["scene_sha256"]:
                problems.append("%s: bins scene_sha256 != summary" % (key,))
                break
        a = agg.get(key)
        if a is not None and a["hash"] != s["rayset_hash"]:
            problems.append("%s: pair-row rayset_hash != summary" % (key,))

    manifest = load_manifest()
    for s in summary:
        info = manifest.get(s["scene"])
        if info is None:
            problems.append("%s: scene missing from scenes.csv" % s["scene"])
        elif info["sha256"] != s["scene_sha256"]:
            problems.append("%s: scene_sha256 != scenes.csv manifest" % s["scene"])

    return problems, rows_seen


def pooled(rows, score):
    c = sum(i(r, score + "_correct") for r in rows)
    p = sum(i(r, "discordant_pairs") for r in rows)
    return ratio(c, p), c, p


def scene_bootstrap(per_scene_delta):
    """Resample scenes with replacement; statistic is the mean scene delta."""
    if not per_scene_delta:
        return None
    rnd = random.Random(SCENE_BOOTSTRAP_SEED)
    n = len(per_scene_delta)
    draws = []
    for _ in range(SCENE_BOOTSTRAP_SAMPLES):
        s = 0.0
        for _ in range(n):
            s += per_scene_delta[rnd.randrange(n)]
        draws.append(s / n)
    draws.sort()
    lo = draws[int(0.025 * len(draws))]
    hi = draws[min(len(draws) - 1, int(0.975 * len(draws)))]
    return sum(per_scene_delta) / n, lo, hi


def table(header, rows):
    out = ["| " + " | ".join(header) + " |",
           "|" + "|".join(["---"] * len(header)) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(x) for x in r) + " |")
    return "\n".join(out)


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2

    run_dir = sys.argv[1]
    summary = read_csv(os.path.join(run_dir, "d1_summary.csv"))
    bins = read_csv(os.path.join(run_dir, "d1_directional_bins.csv"))
    manifest = load_manifest()

    for r in summary:
        info = manifest.get(r["scene"], {})
        r["family"] = info.get("family", "unknown")
        r["role"] = info.get("role", "unknown")
        r["substantive"] = (
            r["role"] == "substantive"
            and i(r, "valid_candidate_events") >= MIN_VALID_EVENTS
            and i(r, "discordant_pairs") >= MIN_PAIRS
        )
        r["support"] = (
            i(r, "valid_candidate_events") >= MIN_VALID_EVENTS
            and i(r, "discordant_pairs") >= MIN_PAIRS
        )

    print("# Direction D aggregation")
    print()
    print("run dir: `%s`" % run_dir)
    print("coordinates: %d" % len(summary))
    print()

    problems, pair_rows = check_reconciliation(summary, bins, open_pairs(run_dir))
    print("## Reconciliation")
    print()
    print("pair rows read: %d" % pair_rows)
    if problems:
        print()
        print("**FAILURES**")
        for p in problems[:50]:
            print("- " + p)
    else:
        print()
        print("All frozen reconciliation identities hold.")
    print()

    # ---------------------------------------------------------------- support
    print("## Per scene x ray set")
    print()
    rows = []
    for r in summary:
        rows.append([
            r["scene"], r["ray_set"], r["ray_seed"], r["width"] + "x" + r["height"],
            r["rays"], r["candidate_events"], r["valid_candidate_events"],
            r["discordant_pairs"], r["rays_with_pairs"],
            pct(f(r, "invalid_rate")), pct(f(r, "saturation_rate")),
            pct(f(r, "false_positive_rate")),
            "yes" if r["support"] else "NO",
        ])
    print(table(["scene", "ray_set", "seed", "res", "rays", "cand", "valid cand",
                 "pairs", "rays w/ pairs", "invalid %", "saturated %", "AABB FP %",
                 "support"], rows))
    print()

    # -------------------------------------------------------------- accuracy
    print("## Within-parent ranking accuracy (%), ties never counted as correct")
    print()
    rows = []
    for r in summary:
        ci = ("[%s, %s]" % (cell(r, "directional_ci_lo", "%.2f"), cell(r, "directional_ci_hi", "%.2f"))
              if has(r, "directional_ci_lo") else "n/a")
        dci = ("[%+.2f, %+.2f]" % (f(r, "delta_ci_lo_pp"), f(r, "delta_ci_hi_pp"))
               if has(r, "delta_ci_lo_pp") else "n/a")
        rows.append([
            r["scene"], r["ray_set"], r["ray_seed"],
            cell(r, "directional_accuracy", "%.2f"), ci,
            cell(r, "surface_density_accuracy", "%.2f"),
            cell(r, "primitive_count_accuracy", "%.2f"),
            cell(r, "box_surface_ratio_accuracy", "%.2f"),
            cell(r, "box_projected_ratio_accuracy", "%.2f"),
            r["best_control"] or "n/a",
            cell(r, "best_control_accuracy", "%.2f"),
            ("%+.2f" % f(r, "delta_pp")) if has(r, "delta_pp") else "n/a",
            dci,
        ])
    print(table(["scene", "ray_set", "seed", "directional", "dir 95% CI",
                 "surf_density", "prim_count", "box_SA", "box_proj",
                 "best control", "best ctl", "delta pp", "delta 95% CI"], rows))
    print()

    print("## Ties per score (count / discordant pairs)")
    print()
    rows = []
    for r in summary:
        row = [r["scene"], r["ray_set"], r["ray_seed"], r["discordant_pairs"]]
        for n in SCORES:
            row.append("%s (%.2f%%)" % (r[n + "_ties"], 100.0 * ratio(i(r, n + "_ties"), i(r, "discordant_pairs"))))
        print_row = row
        rows.append(print_row)
    print(table(["scene", "ray_set", "seed", "pairs"] + SCORES, rows))
    print()

    # ------------------------------------------------------------- calibration
    print("## AABB false-positive rate by Q_raw quintile (valid bins only)")
    print()
    by_coord = {}
    for b in bins:
        by_coord.setdefault(coordinate_key(b), {})[b["ratio_bin"]] = b

    rows = []
    for r in summary:
        bb = by_coord[coordinate_key(r)]
        row = [r["scene"], r["ray_set"], r["ray_seed"]]
        for _, names in QUINTILES:
            c = sum(i(bb[n], "candidate_count") for n in names)
            fp = sum(i(bb[n], "false_positive_count") for n in names)
            row.append("%s (n=%d)" % (pct(ratio(fp, c)) if c else "-", c))
        sat = bb["raw_gt_1"]
        inv = bb["invalid"]
        row.append("%s (n=%s)" % (pct(ratio(i(sat, "false_positive_count"), i(sat, "candidate_count"))) if i(sat, "candidate_count") else "-", sat["candidate_count"]))
        row.append(inv["candidate_count"])
        rows.append(row)
    print(table(["scene", "ray_set", "seed"] + [q[0] for q in QUINTILES] + ["raw_gt_1", "invalid n"], rows))
    print()

    print("## Pooled calibration by bin (all coordinates)")
    print()
    agg = {}
    for b in bins:
        a = agg.setdefault(b["ratio_bin"], {"c": 0, "fp": 0, "pn": 0, "pp": 0, "pb": 0, "pt": 0,
                                            "fn": 0, "fpp": 0, "fb": 0, "ft": 0})
        a["c"] += i(b, "candidate_count")
        a["fp"] += i(b, "false_positive_count")
        a["pn"] += i(b, "probe_node_steps")
        a["pp"] += i(b, "probe_prim_steps")
        a["pb"] += i(b, "probe_box_tests")
        a["pt"] += i(b, "probe_tri_tests")
        a["fn"] += i(b, "false_positive_node_steps")
        a["fpp"] += i(b, "false_positive_prim_steps")
        a["fb"] += i(b, "false_positive_box_tests")
        a["ft"] += i(b, "false_positive_tri_tests")

    order = VALID_BINS + ["raw_gt_1", "invalid"]
    rows = []
    for name in order:
        a = agg.get(name)
        if a is None:
            continue
        rows.append([name, a["c"], a["fp"], pct(ratio(a["fp"], a["c"])),
                     a["pn"], a["pp"], a["pb"], a["pt"],
                     a["fn"], a["fpp"], a["fb"], a["ft"]])
    print(table(["ratio_bin", "candidates", "AABB FP", "FP %",
                 "probe_node_steps", "probe_prim_steps", "probe_box_tests", "probe_tri_tests",
                 "fp_node_steps", "fp_prim_steps", "fp_box_tests", "fp_tri_tests"], rows))
    print()

    # -------------------------------------------------------------- aggregates
    def group_table(title, keyfn):
        print("## Pooled accuracy by %s" % title)
        print()
        groups = {}
        for r in summary:
            groups.setdefault(keyfn(r), []).append(r)
        rows = []
        for key in sorted(groups):
            g = groups[key]
            accs = {n: pooled(g, n)[0] for n in SCORES}
            best = max(CONTROLS, key=lambda n: accs[n])
            pairs = sum(i(r, "discordant_pairs") for r in g)
            cands = sum(i(r, "candidate_events") for r in g)
            fp = sum(i(r, "false_positive_events") for r in g)
            rows.append([key, len(g), cands, pairs,
                         pct(accs["directional"]), best, pct(accs[best]),
                         "%+.2f" % (100.0 * (accs["directional"] - accs[best])),
                         pct(ratio(fp, cands))])
        print(table([title, "rows", "candidates", "pairs", "directional %",
                     "best control", "best ctl %", "delta pp", "AABB FP %"], rows))
        print()

    group_table("scene", lambda r: r["scene"])
    group_table("ray_set", lambda r: r["ray_set"])
    group_table("family", lambda r: r["family"])
    group_table("status", lambda r: ("substantive" if r["substantive"]
                                     else ("diagnostic (development scene)" if r["support"]
                                           else "diagnostic (low support)")))

    # ---------------------------------------------------------- pooled + gates
    print("## Pooled scene-level result")
    print()
    usable = [r for r in summary if r["support"]]
    if not usable:
        print("No row meets the support thresholds.")
        return 0

    accs = {n: pooled(usable, n)[0] for n in SCORES}
    best = max(CONTROLS, key=lambda n: accs[n])
    print("Pooled over %d supported rows: directional %s%%, best control `%s` %s%%, delta %+.2f pp."
          % (len(usable), pct(accs["directional"]), best, pct(accs[best]),
             100.0 * (accs["directional"] - accs[best])))
    print()

    scenes = sorted({r["scene"] for r in usable})
    per_scene = []
    rows = []
    for sc in scenes:
        g = [r for r in usable if r["scene"] == sc]
        a_dir = pooled(g, "directional")[0]
        a_ctl = pooled(g, best)[0]
        per_scene.append(100.0 * (a_dir - a_ctl))
        rows.append([sc, len(g), pct(a_dir), pct(a_ctl), "%+.2f" % (100.0 * (a_dir - a_ctl))])
    print(table(["scene", "rows", "directional %", best + " %", "delta pp"], rows))
    print()

    boot = scene_bootstrap(per_scene)
    if boot:
        mean, lo, hi = boot
        print("Scene bootstrap (seed 0x%X, B=%d, %d scenes): mean delta %+.2f pp, 95%% CI [%+.2f, %+.2f] pp."
              % (SCENE_BOOTSTRAP_SEED, SCENE_BOOTSTRAP_SAMPLES, len(per_scene), mean, lo, hi))
    print()

    # Clause 1 counting, reported for both the formal and the provisional read.
    print("## Gate D1 clause arithmetic")
    print()
    ok_scenes = []
    for sc in scenes:
        g = [r for r in usable if r["scene"] == sc]
        a = pooled(g, "directional")[0]
        ci_lo = min(f(r, "directional_ci_lo") for r in g)
        ok = a >= 0.55 and ci_lo > 0.50
        ok_scenes.append((sc, a, ci_lo, ok))
    print(table(["scene", "pooled directional %", "min row CI lo %", ">=55% and CI>50%"],
                [[s, pct(a), pct(c), "yes" if o else "no"] for s, a, c, o in ok_scenes]))
    print()
    print("clause 1 (>=55%% with CI above 50%% in >=3 scenes): %d of %d scenes qualify"
          % (sum(1 for _, _, _, o in ok_scenes if o), len(ok_scenes)))
    print("clause 2 (>=+3 pp over best control, scene CI above zero): delta %+.2f pp"
          % (100.0 * (accs["directional"] - accs[best])))

    substantive_scenes = sorted({r["scene"] for r in summary if r["substantive"]})
    fams = sorted({r["family"] for r in summary if r["substantive"]})
    print("substantive scenes available: %d %s; families %s"
          % (len(substantive_scenes), substantive_scenes, fams))
    return 0


if __name__ == "__main__":
    sys.exit(main())
