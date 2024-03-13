#include "solve_poisson.hpp"
#include "axes/vdb/poisson.hpp"

using namespace ax;

ax::vdb::RealGridPtr solve_poisson(ax::vdb::RealGridPtr source) {
  using namespace openvdb;
  using namespace openvdb::tools;

  auto bbox_source = source->evalActiveVoxelBoundingBox();
  auto new_source = openvdb::DoubleGrid::create();
  new_source->denseFill(bbox_source, 0.0);
  for (auto iter = source->cbeginValueOn(); iter; ++iter) {
    new_source->tree().setValue(iter.getCoord(), iter.getValue());
  }

  auto solver = vdb::PoissonSolverBase::Create(ax::vdb::PoissonSolverKind::kVdb);
  auto solution = (*solver)(new_source);
  auto solution_tree = solution.value()->treePtr();

  AX_LOG(INFO) << "Solution tree: active=" << solution_tree->activeVoxelCount();
  AX_LOG(INFO) << "Input Tree: active=" << source->tree().activeVoxelCount();
  AX_LOG(INFO) << "bbox size=" << bbox_source.extents().x() * bbox_source.extents().y() * bbox_source.extents().z();

  return solution.value();

  // auto min = ax::math::vec3i(bbox_source.min().x(), bbox_source.min().y(),
  // bbox_source.min().z()); auto max = ax::math::vec3i(bbox_source.max().x(),
  // bbox_source.max().y(), bbox_source.max().z());

  // // Make (max-min) to be power of 2
  // ax::math::vec3i box_size = max - min;
  // idx log2x = ceil(log2(box_size[0] + 1));
  // idx log2y = ceil(log2(box_size[1] + 1));
  // idx log2z = ceil(log2(box_size[2] + 1));
  // idx max_log2 = std::max({log2x, log2y, log2z});
  // AX_LOG(INFO) << "max_log2: " << max_log2;
  // max = min + ax::math::vec3i(1 << max_log2, 1 << max_log2, 1 << max_log2);

  // Octree<real> divergence(min, max, 0);

  // for (auto iter = source->cbeginValueOn(); iter; ++iter) {
  //   auto index = iter.getCoord();
  //   ax::math::vec3i xyz;
  //   xyz.x() = index.x();
  //   xyz.y() = index.y();
  //   xyz.z() = index.z();
  //   divergence.Set(xyz, iter.getValue());
  //   AX_LOG(INFO) << "Set" << index << " to " << iter.getValue();
  // }

  // // Solve Poisson equation, using Gauss Seidel Iteration.
  // using O = Octree<real>;
  // auto solution = divergence.DeepCopy();

  // for (auto iter = source->cbeginValueOn(); iter; ++iter) {
  //   auto index = iter.getCoord();
  //   ax::math::vec3i xyz;
  //   xyz.x() = index.x();
  //   xyz.y() = index.y();
  //   xyz.z() = index.z();
  //   AX_CHECK(solution.GetData(xyz) == iter.getValue());
  // }
  // solution.ForEachLeaf([](O& t) { t.data_ = 0; });

  // for (auto iteration = 0; iteration < 10; ++iteration) {
  //   real du = 0;
  //   solution.ForEachLeaf([&du, &divergence, &solution](O& t) {
  //     ax::math::vec3i box_size = t.BoxSize();
  //     idx box_area = box_size[0] * box_size[1] * box_size[2];
  //     ax::math::vec3i min = t.min_;
  //     real sum = divergence.GetData(min);
  //     for (idx dx : {-1, 1}) {
  //       for (idx dy : {-1, 1}) {
  //         for (idx dz : {-1, 1}) {
  //           ax::math::vec3i nghb_min
  //               = min + (ax::math::vec3i(dx, dy, dz).array() * box_size.array()).matrix();
  //           for (idx i = nghb_min.x(); i < nghb_min.x() + box_size.x(); ++i) {
  //             for (idx j = nghb_min.y(); j < nghb_min.y() + box_size.y(); ++j) {
  //               for (idx k = nghb_min.z(); k < nghb_min.z() + box_size.z(); ++k) {
  //                 ax::math::vec3i xyz(i, j, k);
  //                 if (!solution.IsInside(xyz)) continue;
  //                 real value = solution.GetData(xyz);
  //                 sum += value;
  //               }
  //             }
  //           }
  //         }
  //       }
  //     }
  //     real new_value = sum / (box_area * 6.0);
  //     real original = t.data_;
  //     real delta = new_value - original;
  //     t.data_ = original * 0.5 + new_value * 0.5;
  //     du += delta * delta;

  //     AX_LOG(INFO) << "Box: [" << min.transpose() << "]-[" << t.max_.transpose()
  //                  << "] Area=" << box_area << "Nv=" << new_value << " Orig=" << original
  //                  << " d=" << delta;
  //   });

  //   AX_LOG(INFO) << "Iteration " << iteration << " du: " << du;
  // }
}