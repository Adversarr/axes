#include "ax/geometry/decimate.hpp"

#include "ax/math/linsys/dense/HouseholderQR.hpp"

namespace ax::geo {

struct EdgeCollapseCost {
  HalfedgeEdge* edge;
  math::vec3r target_position;
  real cost;
  bool operator<(EdgeCollapseCost const& other) const {
    return cost > other.cost;
  }
};

MeshDecimator::MeshDecimator(HalfedgeMesh* mesh) : mesh_(mesh), target_count_(mesh->NVertices()) {}

MeshDecimator& MeshDecimator::SetRatio(real ratio) {
  return SetTargetCount(ratio * real(mesh_->NVertices()));
}

MeshDecimator& MeshDecimator::SetTargetCount(idx count) {
  target_count_ = count;
  return *this;
}

real eval_cost(math::mat4r const& Q, math::vec3r const& position) {
  math::vec4r homo;
  homo << position, 1.0;
  return homo.dot(Q * homo);
}

Status MeshDecimator::Run() {
  std::map<HalfedgeVertex*, math::mat4r> Q_i;
  mesh_->ForeachVertex([&](HalfedgeVertex* vert) {
    math::mat4r m = math::mat4r::Zero();
    mesh_->ForeachEdgeAroundVertex(vert, [&](HalfedgeEdge* edge) {
      auto normal = edge->Normal().normalized();
      auto abcd = math::vec4r{normal.x(), normal.y(), normal.z(), -vert->position_.dot(normal)};
      m += abcd * abcd.transpose();
    });
    Q_i[vert] = m;
  });

  List<EdgeCollapseCost> cost;
  mesh_->ForeachEdge([&cost, &Q_i, this](HalfedgeEdge* e) {
    if (!mesh_->CheckCollapse(e)) {
      return;
    }
    EdgeCollapseCost ci;
    ci.edge = e;
    auto const& head = e->vertex_->position_;
    auto const& tail = e->pair_->vertex_->position_;
    auto const& head_Q = Q_i.at(e->vertex_);
    auto const& tail_Q = Q_i.at(e->pair_->vertex_);
    math::mat4r Q = head_Q + tail_Q;

    if (collapse_strategy_ == kDirect) {
      ci.target_position = (head + tail) / 2;
    } else if (collapse_strategy_ == kQuadratic) {
      math::mat4r Q_modified = math::eye<4>();
      Q_modified.topRows<3>() = Q.leftCols<3>().transpose();
      auto qr = Q_modified.colPivHouseholderQr();
      auto solution =  qr.solve(math::vec4r{0, 0, 0, 1}).eval();
      ci.target_position = solution.head<3>();
    }
    ci.cost = eval_cost(Q, ci.target_position);

    cost.push_back(ci);
  });

  std::make_heap(cost.begin(), cost.end());

  while (mesh_->NVertices() > target_count_ && !cost.empty()) {
    EdgeCollapseCost min_cost = cost.front();

    AX_DLOG(INFO) << "Removing edge " << min_cost.edge << " IsBd=" << std::boolalpha << min_cost.edge->IsBoundary()
      << ", " << min_cost.edge->pair_->IsBoundary();
    AX_DLOG(INFO) << "Head: " << min_cost.edge->Head() << "Tail: " << min_cost.edge->Tail();

    if (!mesh_->CheckCollapse(min_cost.edge)) {
      std::pop_heap(cost.begin(), cost.end());
      cost.pop_back();
      continue;
    }

    // Record the edge to collapse
    HalfedgeEdge* edge_to_collapse = min_cost.edge;
    math::vec3r target_position = min_cost.target_position;

    std::set<HalfedgeEdge*> edge_to_remove = {
        edge_to_collapse,
        edge_to_collapse->prev_,
        edge_to_collapse->next_,
        edge_to_collapse->pair_
    };
    if (!edge_to_collapse->pair_->IsBoundary()) {
      edge_to_remove.insert(edge_to_collapse->pair_->prev_);
      edge_to_remove.insert(edge_to_collapse->pair_->next_);
    }

    // Erase the vertex from the Q_i list
    Q_i.erase(edge_to_collapse->Tail());
    auto collapse_vertex = edge_to_collapse->Head();

    // Do the collapse
    mesh_->CollapseEdge(edge_to_collapse, target_position);
    std::set<HalfedgeVertex*> influenced_vertices = {collapse_vertex};
    math::mat4r Q_head = math::zeros<4, 4>();
    mesh_->ForeachEdgeAroundVertex(collapse_vertex, [&](HalfedgeEdge* e) {
      auto normal = e->Normal().normalized();
      auto abcd = math::vec4r{normal.x(), normal.y(), normal.z(),
                              -collapse_vertex->position_.dot(normal)};
      Q_head += abcd * abcd.transpose();
      influenced_vertices.insert(e->pair_->vertex_);
    });
    Q_i[collapse_vertex] = Q_head;
    mesh_->ForeachEdgeAroundVertex(collapse_vertex, [&](HalfedgeEdge* e) {
      const auto v = e->pair_->vertex_;
      math::mat4r Q_v = math::zeros<4, 4>();
      mesh_->ForeachEdgeAroundVertex(v, [&Q_v, &v](HalfedgeEdge* e2) {
        math::vec3r normal = e2->Normal().normalized();
        auto abcd = math::vec4r{normal.x(), normal.y(), normal.z(), -v->position_.dot(normal)};
        Q_v += abcd * abcd.transpose();
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
      } else if (influenced_vertices.contains(c.edge->Head())
                 || influenced_vertices.contains(c.edge->Tail())) {
        const auto e = c.edge;
        auto const& head = e->Head()->position_;
        auto const& tail = e->Tail()->position_;
        auto const& head_Q = Q_i.at(e->Head());
        auto const& tail_Q = Q_i.at(e->Tail());
        math::mat4r Q = head_Q + tail_Q;
        if (collapse_strategy_ == kDirect) {
          c.target_position = (head + tail) / 2;
        } else if (collapse_strategy_ == kQuadratic) {
          math::mat4r Q_modified = math::eye<4>();
          Q_modified.topRows<3>() = Q.leftCols<3>().transpose();
          auto qr = Q_modified.colPivHouseholderQr();
          auto solution = qr.solve(math::vec4r{0, 0, 0, 1}).eval();
          c.target_position = solution.head<3>();
        }
        c.cost = eval_cost(Q, c.target_position);
      }
    }
    std::make_heap(cost.begin(), cost.end());

    AX_CHECK(mesh_->CheckOk());
  }
  AX_RETURN_OK();
}

MeshDecimator& MeshDecimator::SetStrategy(Strategy s) {
  collapse_strategy_ = s;
  return *this;
}

}  // namespace ax::geo
