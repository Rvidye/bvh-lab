#include "direction_d.h"

#include <build/bvh2_builder.h>
#include <core/mode.h>
#include <core/rng.h>
#include <core/traverse_bvh2.h>
#include <eval/directional_geometry.h>
#include <eval/directional_probe.h>
#include <eval/rayset.h>
#include <eval/trace.h>
#include <reference/brute_force.h>
#include <util/camera.h>
#include <util/log.h>
#include <util/metrics.h>
#include <util/timer.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace bvh;

namespace
{
	// ------------------------------------------------------- frozen constants
	//
	// Every value here is fixed by experiments/direction_d/README.md. None of it
	// may be tuned to an observed result.

	constexpr u64 rayset_seed_a = 0x9E3779B97F4A7C15ull;
	constexpr u64 rayset_seed_b = 0xC2B2AE3D27D4EB4Full;

	constexpr u64 bootstrap_seed = 0xD1D1D1D1D1D1D1D1ull;
	constexpr u32 bootstrap_samples = 2000u;

	constexpr u64 min_valid_candidate_events = 10000ull;
	constexpr u64 min_discordant_pairs = 1000ull;

	struct coordinate
	{
		ray_distribution dist;
		u64              seed;
		const char* seed_tag;
	};

	// primary is seed independent, so it is measured once. Duplicating it under a
	// second seed would be a fake replicate.
	const coordinate coordinates[] = {
		{ ray_distribution::primary,    rayset_seed_a, "A" },
		{ ray_distribution::shadow_ao,  rayset_seed_a, "A" },
		{ ray_distribution::shadow_ao,  rayset_seed_b, "B" },
		{ ray_distribution::diffuse_1,  rayset_seed_a, "A" },
		{ ray_distribution::diffuse_1,  rayset_seed_b, "B" },
		{ ray_distribution::incoherent, rayset_seed_a, "A" },
		{ ray_distribution::incoherent, rayset_seed_b, "B" },
	};

	// ------------------------------------------------------------------ sha256

	struct sha256_state
	{
		u32 h[8]{ 0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
				  0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u };
		u64 length{ 0 };
		u8  buffer[64]{};
		u32 buffered{ 0 };
	};

	u32 rotr(u32 v, u32 n) { return (v >> n) | (v << (32u - n)); }

	void sha256_block(sha256_state& s, const u8* p)
	{
		static const u32 k[64] = {
			0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,0x3956c25bu,0x59f111f1u,0x923f82a4u,0xab1c5ed5u,
			0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,
			0xe49b69c1u,0xefbe4786u,0x0fc19dc6u,0x240ca1ccu,0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
			0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,0xc6e00bf3u,0xd5a79147u,0x06ca6351u,0x14292967u,
			0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,
			0xa2bfe8a1u,0xa81a664bu,0xc24b8b70u,0xc76c51a3u,0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
			0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,0x391c0cb3u,0x4ed8aa4au,0x5b9cca4fu,0x682e6ff3u,
			0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u,
		};

		u32 w[64];
		for (u32 i = 0; i < 16; ++i)
			w[i] = (u32(p[i * 4]) << 24) | (u32(p[i * 4 + 1]) << 16) | (u32(p[i * 4 + 2]) << 8) | u32(p[i * 4 + 3]);
		for (u32 i = 16; i < 64; ++i)
		{
			const u32 s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
			const u32 s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
			w[i] = w[i - 16] + s0 + w[i - 7] + s1;
		}

		u32 a = s.h[0], b = s.h[1], c = s.h[2], d = s.h[3];
		u32 e = s.h[4], f = s.h[5], g = s.h[6], hh = s.h[7];

		for (u32 i = 0; i < 64; ++i)
		{
			const u32 S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
			const u32 ch = (e & f) ^ (~e & g);
			const u32 t1 = hh + S1 + ch + k[i] + w[i];
			const u32 S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
			const u32 maj = (a & b) ^ (a & c) ^ (b & c);
			const u32 t2 = S0 + maj;

			hh = g; g = f; f = e; e = d + t1;
			d = c; c = b; b = a; a = t1 + t2;
		}

		s.h[0] += a; s.h[1] += b; s.h[2] += c; s.h[3] += d;
		s.h[4] += e; s.h[5] += f; s.h[6] += g; s.h[7] += hh;
	}

	void sha256_update(sha256_state& s, const u8* data, size_t n)
	{
		s.length += n;
		while (n)
		{
			const size_t take = std::min<size_t>(64 - s.buffered, n);
			std::memcpy(s.buffer + s.buffered, data, take);
			s.buffered += static_cast<u32>(take);
			data += take;
			n -= take;
			if (s.buffered == 64)
			{
				sha256_block(s, s.buffer);
				s.buffered = 0;
			}
		}
	}

	std::string sha256_finish(sha256_state& s)
	{
		const u64 bits = s.length * 8ull;

		const u8 one = 0x80u;
		sha256_update(s, &one, 1);

		const u8 zero = 0u;
		while (s.buffered != 56) sha256_update(s, &zero, 1);

		u8 tail[8];
		for (u32 i = 0; i < 8; ++i) tail[i] = static_cast<u8>((bits >> (56 - i * 8)) & 0xffu);
		sha256_update(s, tail, 8); // s.length is no longer read; bits was captured above

		char out[65];
		for (u32 i = 0; i < 8; ++i) snprintf(out + i * 8, 9, "%08x", s.h[i]);
		out[64] = '\0';
		return out;
	}

	std::string sha256_file(const std::string& path)
	{
		std::ifstream f(path, std::ios::binary);
		if (!f.is_open()) return "unreadable";

		sha256_state s;
		std::vector<char> buf(1 << 16);
		while (f)
		{
			f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
			const std::streamsize got = f.gcount();
			if (got > 0) sha256_update(s, reinterpret_cast<const u8*>(buf.data()), size_t(got));
		}
		return sha256_finish(s);
	}

	// -------------------------------------------------------------- validation

	// The complete ray set against the brute-force oracle. Same policy as the M1
	// harness: a coordinate that cannot be validated publishes no row.
	bool validate_rayset_full(const bvh2& tree, const mesh& m, const rayset& rs, bool any_hit,
		const char* tag, u32 max_report = 5)
	{
		const bvh2_view view = tree.view();
		const auto      prims = make_prims(m, tree);

		u64 mismatches = 0, ties = 0;
		std::vector<u32> scratch;

		for (u32 i = 0; i < rs.size(); ++i)
		{
			const ray r = rs.get(i);

			if (any_hit)
			{
				null_stats s1, s2;
				const bool fast = occluded(view, r, prims, s1);
				const bool slow = occluded_brute_force(m, r, s2);
				if (fast == slow) continue;

				++mismatches;
				if (mismatches <= max_report)
					LOG_ERROR("  ORACLE [%s] anyhit ray=%u: bvh=%d oracle=%d", tag, i, int(fast), int(slow));
				continue;
			}

			hit h;
			h.t = r.t_max;
			null_stats s;
			intersect(view, r, h, prims, s);

			const u32 id = h.valid() ? tree.prim_index(h.id) : invalid_id;
			const oracle_result cmp = compare_against_oracle(m, r, h.valid(), h.t, id, scratch);

			if (cmp.tie_resolved) ++ties;
			if (cmp.agree) continue;

			++mismatches;
			if (mismatches <= max_report)
				LOG_ERROR("  ORACLE [%s] closest ray=%u: hit=%d t=%g id=%u (%s%s%s) |dt|=%g",
					tag, i, int(h.valid()), h.t, id,
					cmp.miss_mismatch ? "hit/miss " : "",
					cmp.t_mismatch ? "t " : "",
					cmp.id_mismatch ? "id" : "", cmp.t_delta);
		}

		if (mismatches)
		{
			LOG_ERROR("  [%s] %llu/%u rays disagree with the oracle", tag,
				(unsigned long long)mismatches, rs.size());
			return false;
		}

		LOG_INFO("  [%s] %u/%u rays agree with the oracle (%llu resolved by tie set)",
			tag, rs.size(), rs.size(), (unsigned long long)ties);
		return true;
	}

	// --------------------------------------------------------------- bootstrap

	// Unbiased index in [0,n) from PCG32, by rejection.
	u32 bounded(rng& g, u32 n)
	{
		const u32 limit = (0xFFFFFFFFu / n) * n;
		u32 v;
		do { v = g.next_u32(); } while (v >= limit);
		return v % n;
	}

	struct interval
	{
		double lo{ 0.0 };
		double hi{ 0.0 };
		bool   valid{ false };
	};

	struct bootstrap_output
	{
		interval accuracy;   // directional
		interval delta;      // directional - best control, paired
	};

	// Ray-clustered nonparametric bootstrap. Rays with no discordant pair are
	// real clusters that contribute (0 correct, 0 pairs), so they are part of the
	// resample even though they are not written to the pairs CSV.
	bootstrap_output ray_cluster_bootstrap(const std::vector<ray_pair_record>& rows,
		u64 total_rays, u32 directional, u32 control)
	{
		bootstrap_output out;
		if (rows.empty() || total_rays == 0) return out;

		const u32 n = static_cast<u32>(std::min<u64>(total_rays, 0xFFFFFFFFull));
		const u32 k = static_cast<u32>(rows.size());

		std::vector<double> acc;
		std::vector<double> delta;
		acc.reserve(bootstrap_samples);
		delta.reserve(bootstrap_samples);

		rng g(bootstrap_seed, 1u);

		for (u32 b = 0; b < bootstrap_samples; ++b)
		{
			u64 pairs = 0, correct_dir = 0, correct_ctl = 0;

			for (u32 draw = 0; draw < n; ++draw)
			{
				const u32 idx = bounded(g, n);
				if (idx >= k) continue;             // a zero-pair ray cluster

				const ray_pair_record& row = rows[idx];
				pairs += row.discordant_pairs;
				correct_dir += row.correct[directional];
				correct_ctl += row.correct[control];
			}

			if (pairs == 0) continue;               // discarded, per the README

			acc.push_back(double(correct_dir) / double(pairs));
			delta.push_back((double(correct_dir) - double(correct_ctl)) / double(pairs));
		}

		auto percentiles = [](std::vector<double>& v, interval& out_iv) {
			if (v.empty()) return;
			std::sort(v.begin(), v.end());
			const size_t lo = size_t(0.025 * double(v.size()));
			size_t hi = size_t(0.975 * double(v.size()));
			if (hi >= v.size()) hi = v.size() - 1;
			out_iv.lo = v[lo];
			out_iv.hi = v[hi];
			out_iv.valid = true;
			};

		percentiles(acc, out.accuracy);
		percentiles(delta, out.delta);
		return out;
	}

	// ------------------------------------------------------------- csv writing

	std::string pairs_header()
	{
		std::string h =
			"run_id,scene,ray_set,ray_seed,rayset_hash,ray_index,query_kind,"
			"candidate_events,discordant_pairs";
		for (u32 s = 0; s < score_count; ++s)
		{
			h += ",";
			h += to_string(static_cast<score_id>(s));
			h += "_correct,";
			h += to_string(static_cast<score_id>(s));
			h += "_ties";
		}
		return h;
	}

	bool append_pair_rows(const std::string& path, const direction_d_args& args,
		const char* ray_set, const char* seed_tag, u64 rayset_hash,
		const char* query_kind, const std::vector<ray_pair_record>& rows)
	{
		const bool is_new = !std::filesystem::exists(path) || std::filesystem::file_size(path) == 0;

		std::ofstream out(path, std::ios::app);
		if (!out.is_open())
		{
			LOG_ERROR("direction_d: cannot open '%s' for append", path.c_str());
			return false;
		}
		if (is_new) out << pairs_header() << '\n';

		char buf[512];
		for (const ray_pair_record& row : rows)
		{
			int n = snprintf(buf, sizeof(buf), "%s,%s,%s,%s,%llu,%u,%s,%u,%u",
				args.run_id.c_str(), args.scene_name.c_str(), ray_set, seed_tag,
				(unsigned long long)rayset_hash, row.ray_index, query_kind,
				row.candidate_events, row.discordant_pairs);
			out.write(buf, n);

			for (u32 s = 0; s < score_count; ++s)
			{
				n = snprintf(buf, sizeof(buf), ",%u,%u", row.correct[s], row.ties[s]);
				out.write(buf, n);
			}
			out.put('\n');
		}

		out.flush();
		if (!out.good())
		{
			LOG_ERROR("direction_d: write failed for '%s'", path.c_str());
			return false;
		}
		return true;
	}

	u64 sum_u64(const u64* a, u32 n)
	{
		u64 s = 0;
		for (u32 i = 0; i < n; ++i) s += a[i];
		return s;
	}

	// ------------------------------------------------------------ one coordinate

	struct measurement
	{
		rayset                      rays;
		directional_analysis_result analysis;
		u32                         width{ 0 };
		u32                         height{ 0 };
		u32                         incoherent_count{ 0 };
		double                      analysis_ms{ 0.0 };
		bool                        support_ok{ false };
	};

	bool support_met(const directional_totals& t)
	{
		const u64 valid_events = t.candidate_events - t.invalid_events;
		return valid_events >= min_valid_candidate_events && t.discordant_pairs >= min_discordant_pairs;
	}

} // namespace

bool run_direction_d(const mesh& original, const direction_d_args& args)
{
	LOG_INFO("=== Direction D: %s ===", args.scene_name.c_str());

	std::filesystem::create_directories(args.run_dir);

	const std::string bins_csv = args.run_dir + "/d1_directional_bins.csv";
	const std::string pairs_csv = args.run_dir + "/d1_directional_pairs.csv";
	const std::string summary_csv = args.run_dir + "/d1_summary.csv";

	const std::string scene_sha = sha256_file(args.scene_path);

	// One canonical tree: binary, robust, binned_sah, max_leaf_size 1.
	mesh m = original;

	build_args ba;
	ba.method = split_method::binned_sah;
	ba.bins = args.bins;
	ba.max_leaf_size = 1;
	ba.silent = false;

	bvh2 tree;
	tree.build(m, ba);
	tree.apply_reorder(m);
	tree.refit(m);

	if (tree.report().max_depth + 2 >= bvh2_stack_size)
	{
		LOG_ERROR("  tree depth %u exceeds the %u-entry traversal stack",
			tree.report().max_depth, bvh2_stack_size);
		return false;
	}

	timer descriptor_timer;
	const std::vector<directional_geometry> geom = compute_directional_geometry(tree, m);
	const double descriptor_ms = descriptor_timer.elapsed_ms();

	LOG_INFO("  tree: %u nodes, depth %u, %u triangles; descriptor %.1f ms",
		tree.report().node_count, tree.report().max_depth, m.triangle_count(), descriptor_ms);

	bool all_ok = true;

	for (const coordinate& coord : coordinates)
	{
		const bool any_hit = (coord.dist == ray_distribution::shadow_ao);
		const probe_query_kind kind = query_kind_for(coord.dist);

		char tag[128];
		snprintf(tag, sizeof(tag), "%s/%s/seed%s", args.scene_name.c_str(),
			to_string(coord.dist), coord.seed_tag);

		// The frozen resolution ladder. A row that misses the support threshold
		// is escalated deterministically, never quietly published as-is.
		u32 res_ladder[3] = { args.width, 1024u, 2048u };
		u32 inc_ladder[2] = { args.incoherent_count, 1048576u };

		const u32 steps = (coord.dist == ray_distribution::incoherent) ? 2u : 3u;

		measurement best;
		bool coordinate_ok = true;

		for (u32 step = 0; step < steps; ++step)
		{
			const u32 res = (coord.dist == ray_distribution::incoherent) ? args.width : res_ladder[step];
			const u32 inc = (coord.dist == ray_distribution::incoherent) ? inc_ladder[step] : args.incoherent_count;

			if (step > 0)
			{
				const bool same = (coord.dist == ray_distribution::incoherent)
					? (inc <= best.incoherent_count)
					: (res <= best.width);
				if (same) continue;
				LOG_WARN("  [%s] support threshold not met; escalating to %s",
					tag, (coord.dist == ray_distribution::incoherent) ? "more rays" : "a higher resolution");
			}

			const camera cam = camera::frame_bounds(m.bounds(), res, res);

			rayset_args ra;
			ra.width = res;
			ra.height = res;
			ra.incoherent_count = inc;
			ra.seed = coord.seed;

			// Generated once, in memory, from the canonical tree. The on-disk
			// rayset cache key is incomplete, so it is never consulted here.
			rayset rs = rayset::generate(coord.dist, m, tree, cam, ra);
			rs.scene = args.scene_name;

			if (rs.empty())
			{
				LOG_ERROR("  [%s] ray set is empty", tag);
				coordinate_ok = false;
				break;
			}

			if (args.validate && !validate_rayset_full(tree, m, rs, any_hit, tag))
			{
				coordinate_ok = false;
				break;
			}

			directional_analysis_args aa;
			aa.query_kind = kind;
			aa.enable_probes = true;
			aa.collect_pair_rows = true;
			aa.collect_events = false;

			timer analysis_timer;
			directional_analysis_result analysis = analyze_rayset(tree, m, geom, rs, aa);
			const double analysis_ms = analysis_timer.elapsed_ms();

			best.width = res;
			best.height = res;
			best.incoherent_count = inc;
			best.analysis_ms = analysis_ms;
			best.support_ok = support_met(analysis.totals);
			best.analysis = std::move(analysis);
			best.rays = std::move(rs);

			if (best.support_ok) break;
		}

		if (!coordinate_ok)
		{
			all_ok = false;
			continue; // no row of any kind for a coordinate we could not trust
		}

		const directional_totals& t = best.analysis.totals;
		const bin_accumulator& b = t.bins;

		// Exact reconciliation before anything is published.
		const bool reconciles =
			sum_u64(b.candidate_count, ratio_bin_count) == t.candidate_events &&
			sum_u64(b.relevant_hit_count, ratio_bin_count) == t.relevant_hit_events &&
			sum_u64(b.false_positive_count, ratio_bin_count) == t.false_positive_events &&
			t.relevant_hit_events + t.false_positive_events == t.candidate_events &&
			b.candidate_count[u32(ratio_bin::invalid)] == t.invalid_events &&
			b.candidate_count[u32(ratio_bin::raw_gt_1)] == t.saturated_events &&
			t.parent_visits == t.trace_node_steps &&
			u64(best.analysis.pair_rows.size()) == t.rays_with_pairs &&
			t.rays == u64(best.rays.size());

		if (!reconciles)
		{
			LOG_ERROR("  [%s] counter reconciliation failed; refusing to publish", tag);
			all_ok = false;
			continue;
		}

		const u64 rayset_hash = best.rays.hash();

		// Point accuracies, then the best cheap control by point accuracy. With no
		// discordant pair there is nothing to be accurate about: every derived
		// column is left empty rather than printed as a fabricated 0.
		const bool has_pairs = t.discordant_pairs > 0;

		double accuracy[score_count];
		for (u32 s = 0; s < score_count; ++s)
			accuracy[s] = has_pairs ? double(t.correct[s]) / double(t.discordant_pairs) : 0.0;

		u32 best_control = u32(score_id::surface_density);
		for (u32 s = u32(score_id::surface_density); s < score_count; ++s)
			if (accuracy[s] > accuracy[best_control]) best_control = s;

		const bootstrap_output boot = ray_cluster_bootstrap(best.analysis.pair_rows, t.rays,
			u32(score_id::directional), best_control);

		if (has_pairs)
			LOG_INFO("  [%s] %llu rays, %llu candidates, %llu pairs, dir %.4f, best control %s %.4f (%.0f ms)",
				tag, (unsigned long long)t.rays, (unsigned long long)t.candidate_events,
				(unsigned long long)t.discordant_pairs, accuracy[u32(score_id::directional)],
				to_string(static_cast<score_id>(best_control)), accuracy[best_control], best.analysis_ms);
		else
			LOG_WARN("  [%s] %llu rays, %llu candidates, NO discordant pair: accuracy is undefined (%.0f ms)",
				tag, (unsigned long long)t.rays, (unsigned long long)t.candidate_events, best.analysis_ms);

		// ------------------------------------------------------------ bins CSV

		bool wrote = true;
		for (u32 i = 0; i < ratio_bin_count; ++i)
		{
			metrics row;
			row.set("run_id", args.run_id);
			row.set("git_commit", args.git_commit);
			row.set("dirty", args.dirty ? 1u : 0u);
			row.set("scene", args.scene_name);
			row.set("scene_sha256", scene_sha);
			row.set("builder", to_string(split_method::binned_sah));
			row.set("layout", "bvh2/slot32_aos");
			row.set("mode", default_mode::name);
			row.set("ray_set", to_string(coord.dist));
			row.set("ray_seed", coord.seed_tag);
			row.set("rayset_hash", std::to_string(rayset_hash));
			row.set("query_kind", to_string(kind));
			row.set("ratio_bin", to_string(static_cast<ratio_bin>(i)));
			row.set("candidate_count", i64(b.candidate_count[i]));
			row.set("relevant_hit_count", i64(b.relevant_hit_count[i]));
			row.set("false_positive_count", i64(b.false_positive_count[i]));
			row.set("invalid_count", i64(i == u32(ratio_bin::invalid) ? b.candidate_count[i] : 0ull));
			row.set("probe_node_steps", i64(b.probe_node_steps[i]));
			row.set("probe_prim_steps", i64(b.probe_prim_steps[i]));
			row.set("probe_box_tests", i64(b.probe_box_tests[i]));
			row.set("probe_tri_tests", i64(b.probe_tri_tests[i]));
			row.set("false_positive_node_steps", i64(b.false_positive_node_steps[i]));
			row.set("false_positive_prim_steps", i64(b.false_positive_prim_steps[i]));
			row.set("false_positive_box_tests", i64(b.false_positive_box_tests[i]));
			row.set("false_positive_tri_tests", i64(b.false_positive_tri_tests[i]));
			// The probe columns are counterfactual child costs, not the work the
			// production traversal performed.
			row.set("source_analytic", "S-analytic");
			row.set("source_steps", "S-probe-counterfactual");
			wrote = row.flush(bins_csv) && wrote;
		}

		// ----------------------------------------------------------- pairs CSV

		wrote = append_pair_rows(pairs_csv, args, to_string(coord.dist), coord.seed_tag,
			rayset_hash, to_string(kind), best.analysis.pair_rows) && wrote;

		// --------------------------------------------------------- summary CSV

		{
			metrics row;
			row.set("run_id", args.run_id);
			row.set("git_commit", args.git_commit);
			row.set("dirty", args.dirty ? 1u : 0u);
			row.set("scene", args.scene_name);
			row.set("scene_sha256", scene_sha);
			row.set("triangles", m.triangle_count());
			row.set("builder", to_string(split_method::binned_sah));
			row.set("bins", args.bins);
			row.set("max_leaf_size", 1u);
			row.set("layout", "bvh2/slot32_aos");
			row.set("mode", default_mode::name);
			row.set("threads", args.threads);
			row.set("nodes", tree.report().node_count);
			row.set("max_depth", tree.report().max_depth);

			row.set("ray_set", to_string(coord.dist));
			row.set("ray_seed", coord.seed_tag);
			row.set("ray_seed_value", std::to_string(coord.seed));
			row.set("rayset_hash", std::to_string(rayset_hash));
			row.set("width", best.width);
			row.set("height", best.height);
			row.set("incoherent_count", best.incoherent_count);
			row.set("query_kind", to_string(kind));
			row.set("validation", args.validate ? "oracle_match" : "skipped");

			row.set("rays", i64(t.rays));
			row.set("rays_with_pairs", i64(t.rays_with_pairs));
			row.set("parent_visits", i64(t.parent_visits));
			row.set("candidate_events", i64(t.candidate_events));
			row.set("candidate_events_in_pair_rows", i64(t.candidate_events_in_pair_rows));
			row.set("valid_candidate_events", i64(t.candidate_events - t.invalid_events));
			row.set("relevant_hit_events", i64(t.relevant_hit_events));
			row.set("false_positive_events", i64(t.false_positive_events));
			row.set("invalid_events", i64(t.invalid_events));
			row.set("saturated_events", i64(t.saturated_events));
			row.set("invalid_rate", t.candidate_events ? double(t.invalid_events) / double(t.candidate_events) : 0.0, 6);
			row.set("saturation_rate", t.candidate_events ? double(t.saturated_events) / double(t.candidate_events) : 0.0, 6);
			row.set("false_positive_rate", t.candidate_events ? double(t.false_positive_events) / double(t.candidate_events) : 0.0, 6);
			row.set("discordant_pairs", i64(t.discordant_pairs));

			for (u32 s = 0; s < score_count; ++s)
			{
				const std::string base = to_string(static_cast<score_id>(s));
				row.set((base + "_correct").c_str(), i64(t.correct[s]));
				row.set((base + "_ties").c_str(), i64(t.ties[s]));
				if (has_pairs) row.set((base + "_accuracy").c_str(), accuracy[s], 6);
				else           row.set((base + "_accuracy").c_str(), "");
			}

			if (boot.accuracy.valid)
			{
				row.set("directional_ci_lo", boot.accuracy.lo, 6);
				row.set("directional_ci_hi", boot.accuracy.hi, 6);
			}
			else
			{
				row.set("directional_ci_lo", "");
				row.set("directional_ci_hi", "");
			}
			row.set("ci_valid", boot.accuracy.valid ? 1u : 0u);

			if (has_pairs)
			{
				row.set("best_control", to_string(static_cast<score_id>(best_control)));
				row.set("best_control_accuracy", accuracy[best_control], 6);
				row.set("delta_pp", 100.0 * (accuracy[u32(score_id::directional)] - accuracy[best_control]), 4);
			}
			else
			{
				row.set("best_control", "");
				row.set("best_control_accuracy", "");
				row.set("delta_pp", "");
			}

			if (boot.delta.valid)
			{
				row.set("delta_ci_lo_pp", 100.0 * boot.delta.lo, 4);
				row.set("delta_ci_hi_pp", 100.0 * boot.delta.hi, 4);
			}
			else
			{
				row.set("delta_ci_lo_pp", "");
				row.set("delta_ci_hi_pp", "");
			}

			row.set("probe_node_steps", i64(sum_u64(b.probe_node_steps, ratio_bin_count)));
			row.set("probe_prim_steps", i64(sum_u64(b.probe_prim_steps, ratio_bin_count)));
			row.set("probe_box_tests", i64(sum_u64(b.probe_box_tests, ratio_bin_count)));
			row.set("probe_tri_tests", i64(sum_u64(b.probe_tri_tests, ratio_bin_count)));
			row.set("false_positive_node_steps", i64(sum_u64(b.false_positive_node_steps, ratio_bin_count)));
			row.set("false_positive_prim_steps", i64(sum_u64(b.false_positive_prim_steps, ratio_bin_count)));
			row.set("false_positive_box_tests", i64(sum_u64(b.false_positive_box_tests, ratio_bin_count)));
			row.set("false_positive_tri_tests", i64(sum_u64(b.false_positive_tri_tests, ratio_bin_count)));

			// The ordinary production counters, on their own scale.
			row.set("trace_node_steps", i64(t.trace_node_steps));
			row.set("trace_prim_steps", i64(t.trace_prim_steps));
			row.set("trace_box_tests", i64(t.trace_box_tests));
			row.set("trace_tri_tests", i64(t.trace_tri_tests));
			row.set("trace_max_stack", t.trace_max_stack);
			row.set("trace_hits", i64(t.trace_hits));

			row.set("support_ok", best.support_ok ? 1u : 0u);
			row.set("support_min_valid_events", i64(min_valid_candidate_events));
			row.set("support_min_pairs", i64(min_discordant_pairs));
			row.set("bootstrap_seed", "0xD1D1D1D1D1D1D1D1");
			row.set("bootstrap_samples", bootstrap_samples);
			row.set("analysis_ms", best.analysis_ms, 1);
			row.set("descriptor_ms", descriptor_ms, 1);
			row.set("source_analytic", "S-analytic");
			row.set("source_probe_steps", "S-probe-counterfactual");
			row.set("source_trace_steps", "S-cpu-analysis-mirror");

			wrote = row.flush(summary_csv) && wrote;
		}

		if (!wrote)
		{
			LOG_ERROR("  [%s] artifact write failed", tag);
			all_ok = false;
		}
	}

	return all_ok;
}
