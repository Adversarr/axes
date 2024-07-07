#include "ax/fem/trimesh.hpp"

#include "ax/core/excepts.hpp"
#include "ax/utils/status.hpp"

#ifdef AX_PLATFORM_WINDOWS
#  undef ERROR
#endif
namespace ax::fem {

template <idx dim> TriMesh<dim>::TriMesh() {}

template <idx dim>
void TriMesh<dim>::SetMesh(element_list_t const& elements, vertex_list_t const& vertices) {
  elements_ = elements;
  vertices_ = vertices;

  idx nV = GetNumVertices(), nE = GetNumElements();
  v_e_map_.clear();
  v_e_map_.resize(static_cast<size_t>(nV));
  for (idx i = 0; i < nE; ++i) {
    auto const& ijk = GetElement(i);
    for (idx d = 0; d < dim + 1; ++d) {
      auto iV = ijk[d];
      ElementPositionPair en_pair{i, d};
      v_e_map_[static_cast<size_t>(iV)].push_back(en_pair);
    }
  }

  ResetAllBoundaries();
}

template <idx dim> void TriMesh<dim>::SetVertices(vertex_list_t const& vertices) {
  AX_THROW_IF_NE(vertices.cols(), vertices_.cols(),
                 "Invalid size: " + std::to_string(vertices.cols())
                     + " != " + std::to_string(vertices_.cols()));
  vertices_ = vertices;
}

template <idx dim> math::vecxr TriMesh<dim>::GetVerticesFlattened() const noexcept {
  math::vecxr vertices_flattened(vertices_.size());
  // for (idx i = 0; i < vertices_.cols(); ++i) {
  //   for (idx j = 0; j < dim; ++j) {
  //     vertices_flattened[i * dim + j] = vertices_(j, i);
  //   }
  // }
  size_t n_dof = static_cast<size_t>(vertices_.size());
  memcpy(vertices_flattened.data(), vertices_.data(), n_dof * sizeof(real));
  return vertices_flattened;
}

template <idx dim> void TriMesh<dim>::ResetBoundary(idx i, idx dof) {
  AX_DCHECK(0 <= i && i < vertices_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < n_dof_per_vertex_) << "Dof out of range.";
  dirichlet_boundary_mask_(dof, i) = 1;
  boundary_values_(dof, i) = 0;
}

template <idx dim> void TriMesh<dim>::MarkDirichletBoundary(idx i, idx dof, const real& value) {
  AX_THROW_IF_TRUE(i < 0 || i >= vertices_.cols(), "Index out of range.");
  AX_THROW_IF_TRUE(dof < 0 || dof >= n_dof_per_vertex_, "Dof out of range.");
  boundary_values_(dof, i) = value;
  dirichlet_boundary_mask_(dof, i) = 0;
}

template <idx dim> void TriMesh<dim>::ResetAllBoundaries() {
  boundary_values_.setZero(n_dof_per_vertex_, vertices_.cols());
  dirichlet_boundary_mask_.setOnes(n_dof_per_vertex_, vertices_.cols());
}

template <idx dim> void TriMesh<dim>::FilterMatrixFull(math::sp_coeff_list const& input,
                                                       math::sp_coeff_list& out) const {
  out.reserve(input.size());
  for (auto& coeff : input) {
    idx row_id = coeff.row() / n_dof_per_vertex_;
    idx row_dof = coeff.row() % n_dof_per_vertex_;
    idx col_id = coeff.col() / n_dof_per_vertex_;
    idx col_dof = coeff.col() % n_dof_per_vertex_;
    if (IsDirichletBoundary(row_id, row_dof) || IsDirichletBoundary(col_id, col_dof)) {
      continue;
    }
    out.push_back(coeff);
  }

  // Add the Dirichlet boundary conditions.
  for (idx i = 0; i < vertices_.cols(); ++i) {
    for (idx j = 0; j < n_dof_per_vertex_; ++j) {
      if (IsDirichletBoundary(i, j)) {
        out.push_back(math::sp_coeff(i * dim + j, i * dim + j, 1));
      }
    }
  }
}

template <idx dim> void TriMesh<dim>::FilterMatrixDof(idx d, math::sp_coeff_list const& input,
                                                      math::sp_coeff_list& out) const {
  out.reserve(input.size());
  for (auto& coeff : input) {
    if (IsDirichletBoundary(coeff.row(), d) || IsDirichletBoundary(coeff.row(), d)) {
      continue;
    }
    out.push_back(coeff);
  }

  // Add the Dirichlet boundary conditions.
  for (idx i = 0; i < vertices_.cols(); ++i) {
    if (IsDirichletBoundary(i, d)) {
      out.push_back(math::sp_coeff(i, i, 1));
    }
  }
}

template <idx dim> void TriMesh<dim>::FilterVector(math::vecxr& inout, bool set_zero) const {
  if (inout.size() != dirichlet_boundary_mask_.cols() * n_dof_per_vertex_) {
    throw std::invalid_argument("Invalid shape.");
  }
  inout.array() *= dirichlet_boundary_mask_.reshaped().array();
  if (!set_zero) {
    inout += boundary_values_.reshaped();
  }
}

template <idx dim>
void TriMesh<dim>::SetNumDofPerVertex(idx n_dof_per_vertex) noexcept {
  n_dof_per_vertex_ = n_dof_per_vertex;
  ResetAllBoundaries();
}

template <idx dim> void TriMesh<dim>::FilterField(math::fieldr<dim>& inout, bool set_zero) const {
  if (math::shape_of(inout) != math::shape_of(dirichlet_boundary_mask_)) {
    AX_LOG(ERROR) << "Invalid shape: " << inout.rows() << "x" << inout.cols()
                  << " != " << dirichlet_boundary_mask_.rows() << "x"
                  << dirichlet_boundary_mask_.cols();
    throw std::invalid_argument("Invalid shape.");
  }
  inout.array() *= dirichlet_boundary_mask_.array();
  if (!set_zero) {
    inout += boundary_values_;
  }
}

template <idx dim> void TriMesh<dim>::FilterMatrixFull(math::spmatr& mat) const {
  for (idx i = 0; i < mat.outerSize(); ++i) {
    for (typename math::spmatr::InnerIterator it(mat, i); it; ++it) {
      idx row_id = it.row() / n_dof_per_vertex_;
      idx row_dof = it.row() % n_dof_per_vertex_;
      idx col_id = it.col() / n_dof_per_vertex_;
      idx col_dof = it.col() % n_dof_per_vertex_;
      if (IsDirichletBoundary(row_id, row_dof) || IsDirichletBoundary(col_id, col_dof)) {
        it.valueRef() = (it.row() == it.col()) ? 1 : 0;
      }
    }
  }

  mat.prune(0, 0);
}

template <idx dim> void TriMesh<dim>::FilterMatrixDof(idx d, math::spmatr& mat) const {
  // math::sp_coeff_list coo;
  // for (idx i = 0; i < mat.outerSize(); ++i) {
  //   for (typename math::spmatr::InnerIterator it(mat, i); it; ++it) {
  //     coo.push_back(math::sp_coeff(it.row(), it.col(), it.value()));
  //   }
  // }
  // math::sp_coeff_list coo_filtered;
  // FilterMatrixDof(d, coo, coo_filtered);
  // mat = math::make_sparse_matrix(GetNumVertices(), GetNumVertices(), coo_filtered);

  for (idx i = 0; i < mat.outerSize(); ++i) {
    for (typename math::spmatr::InnerIterator it(mat, i); it; ++it) {
      if (IsDirichletBoundary(it.row(), d) || IsDirichletBoundary(it.col(), d)) {
        it.valueRef() = (it.row() == it.col()) ? 1 : 0;
      }
    }
  }

  mat.prune(0, 0);
}

template <idx dim> geo::SurfaceMesh TriMesh<dim>::ExtractSurface() const {
  AX_CHECK(dim == 3) << "Only 3D P1Mesh is supported";
  geo::SurfaceMesh surface;
  auto const& elem = this->elements_;
  auto const& vert = this->vertices_;
  if constexpr (dim == 3) {
    surface.vertices_ = vert;
  }
  surface.indices_.resize(3, elem.cols() * 4);
  for (idx i = 0; i < elem.cols(); ++i) {
    auto const& e = elem.col(i);
    surface.indices_.col(4 * i) = math::vec3i{e(0), e(1), e(2)};
    surface.indices_.col(4 * i + 1) = math::vec3i{e(0), e(1), e(3)};
    surface.indices_.col(4 * i + 2) = math::vec3i{e(1), e(2), e(3)};
    surface.indices_.col(4 * i + 3) = math::vec3i{e(2), e(0), e(3)};
  }
  return surface;
}

template <idx dim> void TriMesh<dim>::ApplyPermutation(std::vector<idx> const& perm,
                                                       std::vector<idx> const& inverse_perm) {
  element_list_t elements_new(elements_.rows(), elements_.cols());
  vertex_list_t vertices_new(vertices_.rows(), vertices_.cols());
  for (idx i = 0; i < elements_.cols(); ++i) {
    for (idx j = 0; j < elements_.rows(); ++j) {
      elements_new(j, i) = perm[static_cast<size_t>(elements_(j, i))];
    }
  }
  for (idx i = 0; i < vertices_.cols(); ++i) {
    for (idx j = 0; j < vertices_.rows(); ++j) {
      vertices_new(j, i) = vertices_(j, inverse_perm[static_cast<size_t>(i)]);
    }
  }
  SetMesh(elements_new, vertices_new);
}

template class TriMesh<2>;
template class TriMesh<3>;

}  // namespace ax::fem
