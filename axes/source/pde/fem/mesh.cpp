#include "axes/pde/fem/p1mesh.hpp"

#ifdef AX_PLATFORM_WINDOWS
#undef ERROR
#endif
namespace ax::pde::fem {

template <idx dim> MeshBase<dim>::MeshBase(MeshType type) : type_(type) {}

template <idx dim>
UPtr<MeshBase<dim>> MeshBase<dim>::Create(MeshType type) {
  if (type == MeshType::kP1) {
    return std::make_unique<P1Mesh<dim>>();
  } else {
    AX_LOG(ERROR) << "Unknown mesh type.";
    return nullptr;
  }
  AX_UNREACHABLE();
}

template <idx dim> MeshType MeshBase<dim>::GetType() const noexcept { return type_; }

template <idx dim>
Status MeshBase<dim>::SetMesh(element_list_t const& elements, vertex_list_t const& vertices) {
  if (elements.rows() != GetNumVerticesPerElement()) {
    return utils::InvalidArgumentError("The number of elements does not match.");
  }
  elements_ = elements;
  vertices_ = vertices;

  idx nV = GetNumVertices(), nE = GetNumElements();
  idx nN = GetNumVerticesPerElement();
  v_e_map_.clear();
  v_e_map_.resize(nV);
  for (idx i = 0; i < nE; ++i) {
    auto const& ijk = GetElement(i);
    for (idx d = 0; d < nN; ++d) {
      auto iV = ijk[d];
      ElementPositionPair en_pair{i, d};
      v_e_map_[iV].push_back(en_pair);
    }
  }

  ResetAllBoundaries();
  AX_RETURN_OK();
}

template <idx dim> Status MeshBase<dim>::SetVertices(vertex_list_t const& vertices) {
  if (vertices.size() != vertices_.size()) {
    return utils::InvalidArgumentError("The number of vertices does not match.");
  }
  vertices_ = vertices;
  AX_RETURN_OK();
}

template <idx dim> void MeshBase<dim>::SetVertex(idx i, vertex_t const& vertex) {
  AX_DCHECK(0 <= i && i < vertices_.size()) << "Index out of range.";
  vertices_.col(i) = vertex;
}

template <idx dim> auto MeshBase<dim>::GetVertices() const noexcept
    -> MeshBase<dim>::vertex_list_t const& {
  return vertices_;
}

template <idx dim> 
typename MeshBase<dim>::element_list_t const& MeshBase<dim>::GetElements() const noexcept {
  return elements_;
}

template <idx dim> math::vecxr MeshBase<dim>::GetVerticesFlattened() const noexcept {
  math::vecxr vertices_flattened(vertices_.size());
  for (idx i = 0; i < vertices_.cols(); ++i) {
    for (idx j = 0; j < dim; ++j) {
      vertices_flattened[i * dim + j] = vertices_(j, i);
    }
  }
  return vertices_flattened;
}

template <idx dim> void MeshBase<dim>::ResetBoundary(idx i, idx dof) {
  AX_DCHECK(0 <= i && i < vertices_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < dim) << "Dof out of range.";
  dirichlet_boundary_mask_(dof, i) = 1;
  boundary_values_(dof, i) = 0;
}

template <idx dim> void MeshBase<dim>::MarkDirichletBoundary(idx i, idx dof, const real& value) {
  AX_DCHECK(0 <= i && i < vertices_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < dim) << "Dof out of range.";
  boundary_values_(dof, i) = value;
  dirichlet_boundary_mask_(dof, i) = 0;
}

template <idx dim> void MeshBase<dim>::ResetAllBoundaries() {
  boundary_values_.resize(dim, vertices_.cols());
  dirichlet_boundary_mask_.setOnes(dim, vertices_.cols());
}

template <idx dim>
real MeshBase<dim>::GetBoundaryValue(idx i, idx dof) const noexcept {
  AX_DCHECK(0 <= i && i < boundary_values_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < dim) << "Dof out of range.";
  return boundary_values_(dof, i);
}

template <idx dim> bool MeshBase<dim>::IsDirichletBoundary(idx i, idx dof) const noexcept {
  AX_DCHECK(0 <= i && i < dirichlet_boundary_mask_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < dim) << "Dof out of range.";
  return dirichlet_boundary_mask_(dof, i) == 0;
}

template <idx dim> void MeshBase<dim>::FilterMatrix(math::sp_coeff_list const& input,
                                                    math::sp_coeff_list& out) const {
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

template <idx dim> void MeshBase<dim>::FilterVector(math::vecxr& inout, bool set_zero) const {
  AX_CHECK(inout.rows() == GetNumVertices() * dim) << "Invalid size.";
  inout.array() *= dirichlet_boundary_mask_.reshaped().array();
  if (!set_zero) {
    inout += boundary_values_.reshaped();
  }
}

template <idx dim> void MeshBase<dim>::FilterMatrix(math::sp_matxxr& mat) const {
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

template class MeshBase<2>;
template class MeshBase<3>;

}  // namespace ax::pde::fem