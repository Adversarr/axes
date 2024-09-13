#include "ax/fem/trimesh.hpp"

#include "ax/core/excepts.hpp"

#ifdef AX_PLATFORM_WINDOWS
#  undef ERROR
#endif
namespace ax::fem {

template <int dim> LinearMesh<dim>::LinearMesh() {}

template <int dim>
void LinearMesh<dim>::SetMesh(element_list_t const& elements, vertex_list_t const& vertices) {
  elements_ = elements;
  vertices_ = vertices;

  Index nV = GetNumVertices(), nE = GetNumElements();
  v_e_map_.clear();
  v_e_map_.resize(static_cast<size_t>(nV));
  for (Index i = 0; i < nE; ++i) {
    auto const& ijk = GetElement(i);
    for (Index d = 0; d < dim + 1; ++d) {
      auto iV = ijk[d];
      ElementPositionPair en_pair{i, d};
      v_e_map_[static_cast<size_t>(iV)].push_back(en_pair);
    }
  }

  ResetAllBoundaries();
}

template <int dim> void LinearMesh<dim>::SetVertices(vertex_list_t const& vertices) {
  AX_THROW_IF_NE(vertices.cols(), vertices_.cols(), "Invalid size: {} != {}", vertices.cols(),
                 vertices_.cols());
  vertices_ = vertices;
}

template <int dim> math::RealVectorX LinearMesh<dim>::GetVerticesFlattened() const noexcept {
  math::RealVectorX vertices_flattened(vertices_.size());
  // for (Index i = 0; i < vertices_.cols(); ++i) {
  //   for (Index j = 0; j < dim; ++j) {
  //     vertices_flattened[i * dim + j] = vertices_(j, i);
  //   }
  // }
  size_t n_dof = static_cast<size_t>(vertices_.size());
  memcpy(vertices_flattened.data(), vertices_.data(), n_dof * sizeof(Real));
  return vertices_flattened;
}

template <int dim> void LinearMesh<dim>::ResetBoundary(Index i, Index dof) {
  AX_DCHECK(0 <= i && i < vertices_.cols(), "Index out of range.");
  AX_DCHECK(0 <= dof && dof < n_dof_per_vertex_, "Dof out of range.");
  dirichlet_boundary_mask_(dof, i) = 1;
  boundary_values_(dof, i) = 0;
}

template <int dim> void LinearMesh<dim>::MarkDirichletBoundary(Index i, Index dof, const Real& value) {
  AX_THROW_IF_TRUE(i < 0 || i >= vertices_.cols(), "Index out of range.");
  AX_THROW_IF_TRUE(dof < 0 || dof >= n_dof_per_vertex_, "Dof out of range.");
  boundary_values_(dof, i) = value;
  dirichlet_boundary_mask_(dof, i) = 0;
}

template <int dim> void LinearMesh<dim>::ResetAllBoundaries() {
  boundary_values_.setZero(n_dof_per_vertex_, vertices_.cols());
  dirichlet_boundary_mask_.setOnes(n_dof_per_vertex_, vertices_.cols());
}

template <int dim> void LinearMesh<dim>::FilterMatrixFull(math::RealSparseCOO const& input,
                                                       math::RealSparseCOO& out) const {
  out.reserve(input.size());
  for (auto& coeff : input) {
    Index row_id = coeff.row() / n_dof_per_vertex_;
    Index row_dof = coeff.row() % n_dof_per_vertex_;
    Index col_id = coeff.col() / n_dof_per_vertex_;
    Index col_dof = coeff.col() % n_dof_per_vertex_;
    if (IsDirichletBoundary(row_id, row_dof) || IsDirichletBoundary(col_id, col_dof)) {
      continue;
    }
    out.push_back(coeff);
  }

  // Add the Dirichlet boundary conditions.
  for (Index i = 0; i < vertices_.cols(); ++i) {
    for (Index j = 0; j < n_dof_per_vertex_; ++j) {
      if (IsDirichletBoundary(i, j)) {
        out.push_back(math::RealSparseEntry(i * dim + j, i * dim + j, 1));
      }
    }
  }
}

template <int dim> void LinearMesh<dim>::FilterMatrixDof(Index d, math::RealSparseCOO const& input,
                                                      math::RealSparseCOO& out) const {
  out.reserve(input.size());
  for (auto& coeff : input) {
    if (IsDirichletBoundary(coeff.row(), d) || IsDirichletBoundary(coeff.row(), d)) {
      continue;
    }
    out.push_back(coeff);
  }

  // Add the Dirichlet boundary conditions.
  for (Index i = 0; i < vertices_.cols(); ++i) {
    if (IsDirichletBoundary(i, d)) {
      out.push_back(math::RealSparseEntry(i, i, 1));
    }
  }
}

template <int dim> void LinearMesh<dim>::FilterVector(math::RealVectorX& inout, bool set_zero) const {
  if (inout.size() != dirichlet_boundary_mask_.cols() * n_dof_per_vertex_) {
    throw std::invalid_argument("Invalid shape.");
  }
  inout.array() *= dirichlet_boundary_mask_.reshaped().array();
  if (!set_zero) {
    inout += boundary_values_.reshaped();
  }
}

template <int dim> void LinearMesh<dim>::SetNumDofPerVertex(Index n_dof_per_vertex) noexcept {
  n_dof_per_vertex_ = n_dof_per_vertex;
  ResetAllBoundaries();
}

template <int dim> void LinearMesh<dim>::FilterField(math::RealField<dim>& inout, bool set_zero) const {
  if (math::shape_of(inout) != math::shape_of(dirichlet_boundary_mask_)) {
    AX_ERROR("Invalid Shape: {}x{} != {}x{}", inout.rows(), inout.cols(),
             dirichlet_boundary_mask_.rows(), dirichlet_boundary_mask_.cols());
    throw std::invalid_argument("Invalid shape.");
  }
  inout.array() *= dirichlet_boundary_mask_.array();
  if (!set_zero) {
    inout += boundary_values_;
  }
}

template <int dim> void LinearMesh<dim>::FilterMatrixFull(math::RealSparseMatrix& mat) const {
  for (Index i = 0; i < mat.outerSize(); ++i) {
    for (typename math::RealSparseMatrix::InnerIterator it(mat, i); it; ++it) {
      Index row_id = it.row() / n_dof_per_vertex_;
      Index row_dof = it.row() % n_dof_per_vertex_;
      Index col_id = it.col() / n_dof_per_vertex_;
      Index col_dof = it.col() % n_dof_per_vertex_;
      if (IsDirichletBoundary(row_id, row_dof) || IsDirichletBoundary(col_id, col_dof)) {
        it.valueRef() = (it.row() == it.col()) ? 1 : 0;
      }
    }
  }

  mat.prune(0, 0);
}

template <int dim> void LinearMesh<dim>::FilterMatrixDof(Index d, math::RealSparseMatrix& mat) const {
  // math::sp_coeff_list coo;
  // for (Index i = 0; i < mat.outerSize(); ++i) {
  //   for (typename math::RealSparseMatrix::InnerIterator it(mat, i); it; ++it) {
  //     coo.push_back(math::sp_coeff(it.row(), it.col(), it.value()));
  //   }
  // }
  // math::sp_coeff_list coo_filtered;
  // FilterMatrixDof(d, coo, coo_filtered);
  // mat = math::make_sparse_matrix(GetNumVertices(), GetNumVertices(), coo_filtered);

  for (Index i = 0; i < mat.outerSize(); ++i) {
    for (typename math::RealSparseMatrix::InnerIterator it(mat, i); it; ++it) {
      if (IsDirichletBoundary(it.row(), d) || IsDirichletBoundary(it.col(), d)) {
        it.valueRef() = (it.row() == it.col()) ? 1 : 0;
      }
    }
  }

  mat.prune(0, 0);
}

template <int dim> geo::SurfaceMesh LinearMesh<dim>::ExtractSurface() const {
  AX_CHECK(dim == 3, "Only 3D P1Mesh is supported");
  geo::SurfaceMesh surface;
  auto const& elem = this->elements_;
  auto const& vert = this->vertices_;
  if constexpr (dim == 3) {
    surface.vertices_ = vert;
  }
  surface.indices_.resize(3, elem.cols() * 4);
  for (Index i = 0; i < elem.cols(); ++i) {
    auto const& e = elem.col(i);
    surface.indices_.col(4 * i) = math::IndexVector3{e(0), e(1), e(2)};
    surface.indices_.col(4 * i + 1) = math::IndexVector3{e(0), e(1), e(3)};
    surface.indices_.col(4 * i + 2) = math::IndexVector3{e(1), e(2), e(3)};
    surface.indices_.col(4 * i + 3) = math::IndexVector3{e(2), e(0), e(3)};
  }
  return surface;
}

template <int dim> void LinearMesh<dim>::ApplyPermutation(std::vector<Index> const& perm,
                                                       std::vector<Index> const& inverse_perm) {
  element_list_t elements_new(elements_.rows(), elements_.cols());
  vertex_list_t vertices_new(vertices_.rows(), vertices_.cols());
  for (Index i = 0; i < elements_.cols(); ++i) {
    for (Index j = 0; j < elements_.rows(); ++j) {
      elements_new(j, i) = perm[static_cast<size_t>(elements_(j, i))];
    }
  }
  for (Index i = 0; i < vertices_.cols(); ++i) {
    for (Index j = 0; j < vertices_.rows(); ++j) {
      vertices_new(j, i) = vertices_(j, inverse_perm[static_cast<size_t>(i)]);
    }
  }
  SetMesh(elements_new, vertices_new);
}

template class LinearMesh<2>;
template class LinearMesh<3>;

}  // namespace ax::fem
