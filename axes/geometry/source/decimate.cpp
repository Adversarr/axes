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

void MeshDecimator::Precompute() {
  // Laplace:
  math::sp_coeff_list D_coef;
  idx cnt_d = 0;
  mesh_->ForeachEdge([&](HalfedgeEdge* e) {
    cnt_d += 1;
    D_coef.push_back({cnt_d, e->Tail()->original_id_, 1});
    D_coef.push_back({cnt_d, e->Head()->original_id_, -1});
  });

  math::sp_matxxr D = math::make_sparse_matrix(cnt_d, mesh_->NVertices(), D_coef);
  laplacian_ = D.transpose() * D;
  laplacian_.makeCompressed();

  // Mass: Use Lumped one.
  mass_.setOnes(mesh_->NVertices());
  m_tilde_.setOnes(mesh_->NVertices());

  // Projection, Initialize with identity matrix.
  math::sp_coeff_list P_coef;
  for (idx i = 0; i < mesh_->NVertices(); ++i) {
    P_coef.push_back({i, i, 1});
  }
  projection_matrix_ = math::make_sparse_matrix(mesh_->NVertices(), mesh_->NVertices(), P_coef);

  // Eigen Decomposition
  auto [evec, eval] = math::eig(laplacian_.toDense());
  top_k_eigenvecs_ = evec.rightCols(5);
  Precompute_PlfPrecomputed();
  Precompute_PfPrecomputed();
}

real MeshDecimator::Spectral_RowV(idx original_id) {
  auto v = mesh_->GetVertex(original_id);
  AX_CHECK(static_cast<bool>(v)) << "Cannot find vertex with original id " << original_id;
  // (PM^(-1)LF - M_tilde^(-1)L_tilde P F)
  math::vecxr pmlf = plf_precomputed_.row(original_id).transpose();
  real mtilde = m_tilde_(original_id);
  
  // compute L * PF
  math::vecxr lpf;
  mesh_->ForeachEdgeAroundVertex(v.vertex_, [&](HalfedgeEdge* e) { 
    lpf += pf_precomputed_.row(original_id).transpose();
    lpf -= pf_precomputed_.row(e->Tail()->original_id_).transpose();
  });

  return mtilde * math::norm2(pmlf - lpf / mtilde);
}

real MeshDecimator::Spectral_RowV(idx collapse_a, idx collapse_b) {
  // (PM^(-1)LF - M_tilde^(-1)L_tilde P F)
  auto va = mesh_->GetVertex(collapse_a);
  auto vb = mesh_->GetVertex(collapse_b);

  AX_CHECK(static_cast<bool>(va)) << "Cannot find vertex with original id " << collapse_a;
  AX_CHECK(static_cast<bool>(vb)) << "Cannot find vertex with original id " << collapse_b;

  math::vecxr pmlf = plf_precomputed_.row(collapse_a).transpose();
  real mtilde = m_tilde_(collapse_a);
  
  // compute L * PF
  math::vecxr lpf;
  mesh_->ForeachEdgeAroundVertex(va.vertex_, [&](HalfedgeEdge* e) {
    if (e->Tail()->original_id_ != collapse_b) {
      lpf += pf_precomputed_.row(collapse_a).transpose();
      lpf -= pf_precomputed_.row(e->Tail()->original_id_).transpose();
    }
  });
  mesh_->ForeachEdgeAroundVertex(vb.vertex_, [&](HalfedgeEdge* e) {
    if (e->Tail()->original_id_ != collapse_a) {
      lpf += pf_precomputed_.row(collapse_b).transpose();
      lpf -= pf_precomputed_.row(e->Tail()->original_id_).transpose();
    }
  });

  return mtilde * math::norm2(pmlf - lpf / mtilde);
}

void MeshDecimator::Precompute_PlfPrecomputed() {
  plf_precomputed_ = projection_matrix_ * laplacian_ * top_k_eigenvecs_;
  pf_precomputed_ = projection_matrix_ * top_k_eigenvecs_;
}

real MeshDecimator::EvalCost(idx a, idx b) {
  return Spectral_RowV(a, b) - Spectral_RowV(a) - Spectral_RowV(b);
}

void MeshDecimator::DoCollapse(idx a, idx b) {
  
}

}  // namespace ax::geo
