#include <util/mesh.h>

#include <util/check.h>
#include <util/log.h>
#include <util/timer.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

namespace bvh
{
	bool mesh::load_obj(const std::string& path, f32 scale)
	{
		timer t;

		tinyobj::attrib_t attrib;
		std::vector<tinyobj::shape_t> shapes;
		std::vector<tinyobj::material_t> materials;
		std::string warn, err;

		const bool ok = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.c_str(), nullptr, true);

		if (!warn.empty()) LOG_WARN("obj: %s", warn.c_str());
		if (!err.empty()) LOG_ERROR("obj: %s", err.c_str());
		if (!ok)
		{
			LOG_ERROR("failed to load '%s'\n", path.c_str());
			return false;
		}

		vertices.clear();
		normals.clear();
		tex_coords.clear();
		vertex_indices.clear();
		normal_indices.clear();
		tex_coord_indices.clear();
		material_indices.clear();

		vertices.reserve(attrib.vertices.size() / 3);
		for (size_t i = 0; i + 2 < attrib.vertices.size(); i += 3)
		{
			vertices.emplace_back(	attrib.vertices[i + 0] * scale,
									attrib.vertices[i + 1] * scale,
									attrib.vertices[i + 2] * scale);
		}

		normals.reserve(attrib.normals.size() / 3);
		for (size_t i = 0; i + 2 < attrib.normals.size(); i += 3)
		{
			normals.emplace_back(attrib.normals[i+0], attrib.normals[i+1], attrib.normals[i+2]);
		}

		tex_coords.reserve(attrib.texcoords.size() / 2);
		for (size_t i = 0; i + 1 < attrib.texcoords.size(); i += 2)
			tex_coords.emplace_back(attrib.texcoords[i + 0], attrib.texcoords[i + 1]);

		size_t total_faces = 0;
		for (const tinyobj::shape_t& s : shapes) total_faces += s.mesh.indices.size() / 3;

		vertex_indices.reserve(total_faces);
		normal_indices.reserve(total_faces);
		tex_coord_indices.reserve(total_faces);
		material_indices.reserve(total_faces);

		for (const tinyobj::shape_t& shape : shapes)
		{
			const std::vector<tinyobj::index_t>& idx = shape.mesh.indices;
			CHECK_EQ(idx.size() % 3, size_t(0));

			for (size_t f = 0; f + 2 < idx.size(); f+= 3)
			{
				auto attr = [](int i) { return i < 0 ? invalid_id : static_cast<u32>(i); };

				vertex_indices.emplace_back( attr(idx[f + 0].vertex_index),
											 attr(idx[f + 1].vertex_index),
											 attr(idx[f + 2].vertex_index));

				normal_indices.emplace_back( attr(idx[f + 0].normal_index),
											 attr(idx[f + 1].normal_index),
											 attr(idx[f + 2].normal_index));

				tex_coord_indices.emplace_back( attr(idx[f + 0].texcoord_index),
												attr(idx[f + 1].texcoord_index),
												attr(idx[f + 2].texcoord_index));

				const size_t face = f / 3;
				const int mat = face < shape.mesh.material_ids.size() ? shape.mesh.material_ids[face] : -1;
				material_indices.push_back(mat < 0 ? invalid_id : static_cast<u32>(mat));
			}
		}

		compute_bounds();
		LOG_INFO("loaded %s: %u tris, %zu verts, %zu normals (%.0f ms)",  path.c_str(), triangle_count(), vertices.size(), normals.size(), t.elapsed_ms());
		return !vertex_indices.empty();
	}

	void mesh::compute_bounds()
	{
		_bounds = aabb();
		for (const vec3& v : vertices) _bounds.add(v);
	}

	vec3 mesh::shading_normal(u32 i, const vec2& bc) const
	{
		const uvec3& ni = normal_indices[i];
		if (ni.x == invalid_id || ni.y == invalid_id || ni.z == invalid_id || normals.empty())
			return geometric_normal(i);

		const f32 w = 1.0f - bc.x - bc.y;
		return normalize(normals[ni.x] * bc.x + normals[ni.y] * bc.y + normals[ni.z] * w);
	}

	void mesh::reorder(const std::vector<u32>& permutation)
	{
		CHECK_EQ(permutation.size(), size_t(triangle_count()));

		const std::vector<uvec3> old_vi = vertex_indices;
		const std::vector<uvec3> old_ni = normal_indices;
		const std::vector<uvec3> old_ti = tex_coord_indices;
		const std::vector<u32> old_mi = material_indices;

		for (u32 i = 0; i < permutation.size(); ++i)
		{
			const u32 src = permutation[i];
			DCHECK_LT(src, triangle_count());
			vertex_indices[i] = old_vi[src];
			normal_indices[i] = old_ni[src];
			tex_coord_indices[i] = old_ti[src];
			material_indices[i] = old_mi[src];
		}
	}
} // namespace bvh

