#pragma once

#include <bvh.h>
#include <core/aabb.h>
#include <core/bvh2.h>
#include <util/mesh.h>

#include <vector>

namespace bvh
{

	enum class split_method
	{
		// Split at the longest axis, fast, poor quality tree.
		median,

		sweep_sah,

		binned_sah,

		binned_sah_arches,
	};

	const char* to_string(split_method m);

	struct build_args
	{
		split_method method{ split_method::binned_sah };
		u32 bins{ 32 };
		u32 max_leaf_size{ 1 };
		bool silent{ false };
	};

	struct build_report
	{
		double build_ms{ 0.0 };
		u32 node_count{ 0 };
		u32 leaf_count{ 0 };
		u32 interior_count{ 0 };
		u32 max_depth{ 0 };
		u32 max_leaf_size{ 0 };
		double mean_leaf_size{ 0.0 };
	};

	class bvh2
	{
	public:

		void build(const mesh& m, const build_args& args = {});

		// apply build permutation to the mesh
		void apply_reorder(mesh& m);

		// recompute every node's bound bottom up from current geometry
		void refit(const mesh& m);

		bvh2_view view() const { return bvh2_view{_nodes.data(), static_cast<u32>(_nodes.size())}; }

		const std::vector<bvh2_node>& nodes() const { return _nodes; }
		const std::vector<u32>& prim_indices() const { return _prim_indices; }

		u32 prim_index(u32 slot) const { return _prim_indices[slot]; }

		const build_report& report() const { return _report; }
		const build_args& args() const { return _args; }

		bool empty() const { return _nodes.empty(); }

		// Recomputes depth/leaf statistics from the current tree.
		build_report compute_report() const;

	private:
		std::vector<bvh2_node> _nodes;
		std::vector<u32> _prim_indices;
		build_args _args{};
		build_report _report{};
	};
} // namespace bvh


