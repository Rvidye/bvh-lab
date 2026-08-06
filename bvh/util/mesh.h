#pragma once

#include <bvh.h>
#include <core/aabb.h>
#include <core/ray.h>
#include <core/vec.h>

#include <string>
#include <vector>

namespace bvh
{
	// triagnle mesh in structu-of-arrays form

	class mesh
	{

	public:

		bool load_obj(const std::string& path, f32 scale = 1.0f);

		u32 triangle_count() const { return static_cast<u32>(vertex_indices.size()); }
		bool empty() const { return vertex_indices.empty(); }

		triangle get_triangle(u32 i) const
		{
			const uvec3& vi = vertex_indices[i];
			return triangle{vertices[vi.x], vertices[vi.y], vertices[vi.z]};
		}

		aabb triangle_bounds(u32 i) const { return get_triangle(i).bounds(); }
		vec3 triangle_centroid(u32 i) const { return get_triangle(i).centroid(); }

		vec3 geometric_normal(u32 i) const { return get_triangle(i).normal(); }

		vec3 shading_normal(u32 i, const vec2& bc) const;

		const aabb& bounds() const { return _bounds; }
		void compute_bounds();

		// permutaion[new_index] == old_index
		void reorder(const std::vector<u32>& permutation);

		// per-vertex data
		std::vector<vec3> vertices;
		std::vector<vec3> normals;
		std::vector<vec2> tex_coords;

		// per-triangle data
		std::vector<uvec3> vertex_indices;
		std::vector<uvec3> normal_indices;
		std::vector<uvec3> tex_coord_indices;
		std::vector<u32> material_indices;

	private:
		aabb _bounds;
	};

}


