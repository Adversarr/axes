#include "ax/fem/trimesh.hpp"

#include "ax/utils/status.hpp"

#ifdef AX_PLATFORM_WINDOWS
#  undef ERROR
#endif
namespace ax::fem {

template <idx dim> TriMesh<dim>::TriMesh() {}

template <idx dim>
Status TriMesh<dim>::SetMesh(element_list_t const& elements, vertex_list_t const& vertices) {
  elements_ = elements;
  vertices_ = vertices;

  idx nV = GetNumVertices(), nE = GetNumElements();
  v_e_map_.clear();
  v_e_map_.resize(nV);
  for (idx i = 0; i < nE; ++i) {
    auto const& ijk = GetElement(i);
    for (idx d = 0; d < dim + 1; ++d) {
      auto iV = ijk[d];
      ElementPositionPair en_pair{i, d};
      v_e_map_[iV].push_back(en_pair);
    }
  }

  ResetAllBoundaries();
  AX_RETURN_OK();
}

template <idx dim> Status TriMesh<dim>::SetVertices(vertex_list_t const& vertices) {
  if (vertices.size() != vertices_.size()) {
    return utils::InvalidArgumentError("The number of vertices does not match.");
  }
  vertices_ = vertices;
  AX_RETURN_OK();
}

template <idx dim> void TriMesh<dim>::SetVertex(idx i, vertex_t const& vertex) {
  AX_DCHECK(0 <= i && i < vertices_.size()) << "Index out of range.";
  vertices_.col(i) = vertex;
}

template <idx dim> auto TriMesh<dim>::GetVertices() const noexcept
    -> TriMesh<dim>::vertex_list_t const& {
  return vertices_;
}

template <idx dim>
typename TriMesh<dim>::element_list_t const& TriMesh<dim>::GetElements() const noexcept {
  return elements_;
}

template <idx dim> math::vecxr TriMesh<dim>::GetVerticesFlattened() const noexcept {
  math::vecxr vertices_flattened(vertices_.size());
  for (idx i = 0; i < vertices_.cols(); ++i) {
    for (idx j = 0; j < dim; ++j) {
      vertices_flattened[i * dim + j] = vertices_(j, i);
    }
  }
  return vertices_flattened;
}

template <idx dim> void TriMesh<dim>::ResetBoundary(idx i, idx dof) {
  AX_DCHECK(0 <= i && i < vertices_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < dim) << "Dof out of range.";
  dirichlet_boundary_mask_(dof, i) = 1;
  boundary_values_(dof, i) = 0;
}

template <idx dim> void TriMesh<dim>::MarkDirichletBoundary(idx i, idx dof, const real& value) {
  AX_DCHECK(0 <= i && i < vertices_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < dim) << "Dof out of range.";
  boundary_values_(dof, i) = value;
  dirichlet_boundary_mask_(dof, i) = 0;
}

template <idx dim> void TriMesh<dim>::ResetAllBoundaries() {
  boundary_values_.resize(dim, vertices_.cols());
  boundary_values_.setZero();
  dirichlet_boundary_mask_.setOnes(dim, vertices_.cols());
}

template <idx dim> real TriMesh<dim>::GetBoundaryValue(idx i, idx dof) const noexcept {
  AX_DCHECK(0 <= i && i < boundary_values_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < dim) << "Dof out of range.";
  return boundary_values_(dof, i);
}

template <idx dim> bool TriMesh<dim>::IsDirichletBoundary(idx i, idx dof) const noexcept {
  AX_DCHECK(0 <= i && i < dirichlet_boundary_mask_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < dim) << "Dof out of range.";
  return dirichlet_boundary_mask_(dof, i) == 0;
}

template <idx dim>
void TriMesh<dim>::FilterMatrix(math::sp_coeff_list const& input, math::sp_coeff_list& out) const {
  out.reserve(input.size());
  for (auto& coeff : input) {
    idx row_id = coeff.row() / dim;
    idx row_dof = coeff.row() % dim;
    idx col_id = coeff.col() / dim;
    idx col_dof = coeff.col() % dim;
    if (IsDirichletBoundary(row_id, row_dof) || IsDirichletBoundary(col_id, col_dof)) {
      continue;
    }
    out.push_back(coeff);
  }

  // Add the Dirichlet boundary conditions.
  for (idx i = 0; i < vertices_.cols(); ++i) {
    for (idx j = 0; j < dim; ++j) {
      if (IsDirichletBoundary(i, j)) {
        out.push_back(math::sp_coeff(i * dim + j, i * dim + j, 1));
      }
    }
  }
}

template <idx dim> void TriMesh<dim>::FilterVector(math::vecxr& inout, bool set_zero) const {
  AX_CHECK(inout.rows() == GetNumVertices() * dim) << "Invalid size.";
  inout.array() *= dirichlet_boundary_mask_.reshaped().array();
  if (!set_zero) {
    inout += boundary_values_.reshaped();
  }
}

template <idx dim> void TriMesh<dim>::FilterMatrix(math::sp_matxxr& mat) const {
  math::sp_coeff_list coo;
  for (idx i = 0; i < mat.outerSize(); ++i) {
    for (typename math::sp_matxxr::InnerIterator it(mat, i); it; ++it) {
      coo.push_back(math::sp_coeff(it.row(), it.col(), it.value()));
    }
  }
  math::sp_coeff_list coo_filtered;
  FilterMatrix(coo, coo_filtered);
  mat = math::make_sparse_matrix(dim * GetNumVertices(), dim * GetNumVertices(), coo_filtered);
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
      elements_new(j, i) = perm[elements_(j, i)];
    }
  }
  for (idx i = 0; i < vertices_.cols(); ++i) {
    for (idx j = 0; j < vertices_.rows(); ++j) {
      vertices_new(j, i) = vertices_(j, inverse_perm[i]);
    }
  }
  AX_CHECK_OK(SetMesh(elements_new, vertices_new));
}

template class TriMesh<2>;
template class TriMesh<3>;

}  // namespace ax::fem
