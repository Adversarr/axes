#include "ax/xpbd/global_step_collision_free.hpp"

#include "ax/geometry/intersection/vertex_face.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {

using namespace geo;

void global_step_collision_free(math::field3r const& weighted_position,
                                math::field1r const& weights) {
  auto& g = ensure_server();
  idx const nV = g.last_vertices_.cols();
  // foreach vertex:
  math::field1r min_toi(1, nV);
  min_toi.setOnes();
  math::field3r expect_position = weighted_position;
  for (auto i : utils::iota(nV)) {
    expect_position.col(i) /= weights(i);
  }
  expect_position.swap(g.vertices_);
  //
  // // detect the collision.
  // bool collision_free = true;
  // for (auto i: utils::iota(nV)) {
  //   for (auto [j, f]: utils::enumerate(g.faces_)) {
  //     if (i == f[0] || i == f[1] || i == f[2]) continue;
  //
  //     CollidableVertex vi_prev(i, Vertex3{g.last_vertices_.col(i)});
  //     CollidableVertex vi_next(i, Vertex3{expect_position.col(i)});
  //     CollidableTriangle fj_prev(j, Triangle3{g.last_vertices_.col(f[0]),
  //     g.last_vertices_.col(f[1]), g.last_vertices_.col(f[2])}); CollidableTriangle fj_next(j,
  //     Triangle3{expect_position.col(f[0]), expect_position.col(f[1]),
  //     expect_position.col(f[2])});
  //
  //
  //     // BUG: if vertex is taking off, the collision will also be detected.
  //     if (auto collision = detect_vertex_face(vi_prev, vi_next, fj_prev, fj_next, 1e-5)) {
  //       for (idx v: {i, f[0], f[1], f[2]}) {
  //         min_toi(v) = std::min(min_toi(v), collision.rel_t_);
  //       }
  //       AX_LOG(ERROR) << "Collision detected: " << collision.rel_t_ << " at vertex " << i << "
  //       and face " << j; collision_free = false;
  //     }
  //   }
  // }
  //
  // if (!collision_free) {
  //   std::cout << min_toi << std::endl;
  // }
  //
  // // update the position.
  // for (auto i: utils::iota(nV)) {
  //   expect_position.col(i) = math::lerp(g.last_vertices_.col(i), expect_position.col(i),
  //   min_toi(i));
  // }
  // g.vertices_.swap(expect_position);
}

}  // namespace ax::xpbd
