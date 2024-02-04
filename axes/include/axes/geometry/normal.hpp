#pragma once
#include "axes/math/common.hpp"
namespace ax::geo {

namespace details {
struct face_uniform_avg_t {};
struct face_area_avg_t {};
struct face_angle_avg_t {};
}  // namespace details

constexpr details::face_uniform_avg_t face_uniform_avg{};
constexpr details::face_area_avg_t face_area_avg{};
constexpr details::face_angle_avg_t face_angle_avg{};

math::field3r normal_per_face(math::field3r const& vertices, math::field3i const& indices);

math::field3r normal_per_vertex(math::field3r const& vertices, math::field3i const& indices,
                                details::face_uniform_avg_t = {});

}  // namespace ax::geo
