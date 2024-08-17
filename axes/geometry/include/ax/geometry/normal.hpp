#pragma once
#include "ax/math/common.hpp"
namespace ax::geo {

namespace details {
struct face_uniform_avg_t {};
struct face_area_avg_t {};
struct face_angle_avg_t {};
}  // namespace details

constexpr details::face_uniform_avg_t face_uniform_avg{};
constexpr details::face_area_avg_t face_area_avg{};
constexpr details::face_angle_avg_t face_angle_avg{};

math::RealField3 normal_per_face(math::RealField3 const& vertices, math::IndexField3 const& indices);

math::RealField3 normal_per_vertex(math::RealField3 const& vertices, math::IndexField3 const& indices,
                                details::face_uniform_avg_t = {});

math::RealField3 normal_per_vertex(math::RealField3 const& vertices, math::IndexField3 const& indices,
                                details::face_area_avg_t);

math::RealField3 normal_per_vertex(math::RealField3 const& vertices, math::IndexField3 const& indices,
                                details::face_angle_avg_t);

}  // namespace ax::geo
