#include "axes/geometry/decimate.hpp"

namespace ax::geo {

MeshDecimator::MeshDecimator(HalfedgeMesh* mesh) : mesh_(mesh), target_count_(mesh->NVertices()) {}

MeshDecimator& MeshDecimator::SetRatio(real ratio) {
  return SetTargetCount(ratio * mesh_->NVertices());
}

MeshDecimator& MeshDecimator::SetTargetCount(idx count) {
  target_count_ = count;
  return *this;
}

Status MeshDecimator::Run() {
  std::map<HalfedgeVertex_t*, math::mat3r> Q_i;
  mesh_->ForeachVertex([&](HalfedgeVertex_t* vert) {
    math::mat3r m = math::mat3r::Zero();
    mesh_->ForeachEdgeAroundVertex(vert, [&](HalfedgeEdge_t* edge) {
      auto normal = edge->Normal();
      m += normal * normal.transpose();
    });
    Q_i[vert] = m;
  });

  std::vector<EdgeCollapseCost> cost;
  mesh_->ForeachEdge([&cost, &Q_i, this](HalfedgeEdge_t* e) {
    if (!mesh_->CheckCollapse(e)) {
      return;
    }
    EdgeCollapseCost ci;
    ci.edge = e;
    auto head = e->vertex_->position_;
    auto tail = e->pair_->vertex_->position_;
    math::mat3r head_Q = Q_i.at(e->vertex_);
    math::mat3r tail_Q = Q_i.at(e->pair_->vertex_);
    // TODO: Replace with a better one
    ci.target_position = (head + tail) / 2;
    ci.cost = ci.target_position.dot(((head_Q + tail_Q) * ci.target_position));

    cost.push_back(ci);
  });

  std::make_heap(cost.begin(), cost.end());

  while (mesh_->NVertices() > target_count_ && !cost.empty()) {
    EdgeCollapseCost min_cost = cost.front();
    if (!mesh_->CheckCollapse(min_cost.edge)) {
      std::pop_heap(cost.begin(), cost.end());
      cost.pop_back();
      continue;
    }
    HalfedgeEdge_t* edge_to_collapse = min_cost.edge;
    math::vec3r target_position = min_cost.target_position;

    std::set<HalfedgeEdge_t*> edge_to_remove
        = {edge_to_collapse,        edge_to_collapse->prev_,        edge_to_collapse->next_,
           edge_to_collapse->pair_, edge_to_collapse->pair_->prev_, edge_to_collapse->pair_->next_};
    Q_i.erase(edge_to_collapse->prev_->vertex_);
    auto collapse_vertex = edge_to_collapse->vertex_;
    mesh_->CollapseEdge(edge_to_collapse, target_position);
    // mesh_->CheckOk();
    std::set<HalfedgeVertex_t*> influenced_vertices = {collapse_vertex};
    math::mat3r Q_head = math::zeros<3, 3>();
    mesh_->ForeachEdgeAroundVertex(collapse_vertex, [&](HalfedgeEdge_t* e) {
      auto normal = e->Normal();
      Q_head += normal * normal.transpose();
      influenced_vertices.insert(e->pair_->vertex_);
    });
    Q_i[collapse_vertex] = Q_head;
    mesh_->ForeachEdgeAroundVertex(collapse_vertex, [&](HalfedgeEdge_t* e) {
      auto v = e->pair_->vertex_;
      math::mat3r Q_v = math::zeros<3, 3>();
      mesh_->ForeachEdgeAroundVertex(v, [&Q_v](HalfedgeEdge_t* e2) {
        auto normal = e2->Normal();
        Q_v += normal * normal.transpose();
      });
      Q_i[v] = Q_v;
    });
    // Vertex Position is updated, need to update the cost list:
    for (size_t i = 0; i < cost.size(); ++i) {
      auto& c = cost[i];
      if (edge_to_remove.contains(c.edge) || !mesh_->CheckCollapse(c.edge)) {
        std::swap(c, cost.back());
        cost.pop_back();
        --i;
      } else if (influenced_vertices.contains(c.edge->vertex_)
                 || influenced_vertices.contains(c.edge->pair_->vertex_)) {
        auto head = c.edge->vertex_->position_;
        auto tail = c.edge->pair_->vertex_->position_;
        c.target_position = (head + tail) / 2;
        auto Q_head_update = Q_i.at(c.edge->vertex_), Q_tail = Q_i.at(c.edge->pair_->vertex_);
        c.cost = c.target_position.dot(((Q_head_update + Q_tail) * c.target_position));
      }
    }
    std::make_heap(cost.begin(), cost.end());
  }
  AX_RETURN_OK();
}

}  // namespace ax::geo