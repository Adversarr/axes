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
}