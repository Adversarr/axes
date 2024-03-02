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
      auto A = edge->vertex_->position_;
      auto B = edge->next_->vertex_->position_;
      auto C = edge->next_->next_->vertex_->position_;
      auto normal = (B - A).cross(C - A);
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

  while (mesh_->NVertices() > target_count_ && ! cost.empty()) {
    EdgeCollapseCost min_cost = cost.front();
    if (min_cost.edge == nullptr || !mesh_->CheckCollapse(min_cost.edge)) {
      std::pop_heap(cost.begin(), cost.end());
      cost.pop_back();
      continue;
    }

    mesh_->ForeachEdge([&min_cost, &Q_i, this](HalfedgeEdge_t* e) {
      if (!mesh_->CheckCollapse(e)) {
        return;
      }
      EdgeCollapseCost ci;
      ci.edge = e;
      auto head = e->vertex_->position_;
      auto tail = e->pair_->vertex_->position_;
      ci.target_position = (head + tail) / 2;
      math::mat3r head_Q = Q_i.at(e->vertex_);
      math::mat3r tail_Q = Q_i.at(e->pair_->vertex_);
      ci.cost = ci.target_position.dot(((head_Q + tail_Q) * ci.target_position));
      if (ci.cost < min_cost.cost) {
        min_cost = ci;
      }
    });

    HalfedgeEdge_t* edge_to_collapse = min_cost.edge;
    math::vec3r target_position = min_cost.target_position;

    std::set<HalfedgeEdge_t*> edge_to_remove
        = {edge_to_collapse,        edge_to_collapse->prev_,        edge_to_collapse->next_,
           edge_to_collapse->pair_, edge_to_collapse->pair_->prev_, edge_to_collapse->pair_->next_};
    Q_i.erase(edge_to_collapse->prev_->vertex_);
    auto collapse_vertex = edge_to_collapse->vertex_;
    mesh_->CollapseEdge(edge_to_collapse, target_position);
    // mesh_->CheckOk();
    std::set<HalfedgeVertex_t* > influenced_vertices = {collapse_vertex};
    math::mat3r Q_head = math::zeros<3, 3>();
    mesh_->ForeachEdgeAroundVertex(collapse_vertex, [&](HalfedgeEdge_t* e) {
      auto A = e->vertex_->position_;
      auto B = e->next_->vertex_->position_;
      auto C = e->next_->next_->vertex_->position_;
      auto normal = (B - A).cross(C - A);
      Q_head += normal * normal.transpose();
      influenced_vertices.insert(e->vertex_);
    });
    Q_i[collapse_vertex] = Q_head;
    mesh_->ForeachEdgeAroundVertex(collapse_vertex, [&](HalfedgeEdge_t* e) {
      auto v = e->pair_->vertex_;
      math::mat3r Q_v = math::zeros<3, 3>();
      mesh_->ForeachEdgeAroundVertex(v, [&Q_v](HalfedgeEdge_t* e2) {
        auto A = e2->vertex_->position_;
        auto B = e2->next_->vertex_->position_;
        auto C = e2->next_->next_->vertex_->position_;
        auto normal = (B - A).cross(C - A);
        Q_v += normal * normal.transpose();
      });
      Q_i[v] = Q_v;
    });
    // Vertex Position is updated, need to update the cost list:
    size_t nullptr_count = 0;
    for (size_t i = 0; i < cost.size(); ++i) {
      auto& c = cost[i];
      if (c.edge == nullptr) {
        continue;
      } else if (edge_to_remove.contains(c.edge) || !mesh_->CheckCollapse(c.edge)) {
        c.edge = nullptr;
      } else if (influenced_vertices.contains(c.edge->vertex_) || influenced_vertices.contains(c.edge->pair_->vertex_)){
        auto head = c.edge->vertex_->position_;
        auto tail = c.edge->pair_->vertex_->position_;
        EdgeCollapseCost c_new;
        c_new.edge = c.edge;
        c_new.target_position = (head + tail) / 2;
        auto Q_head = Q_i.at(c_new.edge->vertex_), Q_tail = Q_i.at(c_new.edge->pair_->vertex_);
        c_new.cost = c_new.target_position.dot(((Q_head + Q_tail) * c_new.target_position));
        c.edge = nullptr;
        cost.push_back(c_new);
        std::push_heap(cost.begin(), cost.end());
      }
    }
    if (nullptr_count > cost.size() / 2) {
      auto new_cost = std::vector<EdgeCollapseCost>();
      for (auto& c : cost) {
        if (c.edge != nullptr) {
          new_cost.push_back(c);
        }
      }
      cost = std::move(new_cost);
      std::make_heap(cost.begin(), cost.end());
    }
  }
  AX_RETURN_OK();
}

}  // namespace ax::geo