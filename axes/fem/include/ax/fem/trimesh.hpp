#pragma once

#include "ax/core/echo.hpp"
#include "ax/geometry/common.hpp"
#include "ax/math/sparse.hpp"
#include "ax/core/echo.hpp"

namespace ax::fem {

/**
 * @brief A base class for mesh representation in finite element method.
 *
 * This class provides a base implementation for representing a mesh in the finite element method.
 * It defines the data structures and methods necessary for handling the mesh elements, vertices,
 * and boundary conditions.
 *
 * @tparam dim The dimension of the mesh.
 */
template <idx dim> class TriMesh final {
public:
  using element_t = math::veci<dim + 1>;        /**< Type for storing mesh elements. */
  using element_list_t = math::fieldi<dim + 1>; /**< Type for storing a list of mesh elements. */

  using vertex_t = math::vecr<dim>;        /**< Type for storing mesh vertices. */
  using vertex_list_t = math::fieldr<dim>; /**< Type for storing a list of mesh vertices. */

  using boundary_value_t = vertex_t;           /**< Type for storing boundary values. */
  using boundary_value_list_t = math::fieldxr; /**< Type for storing a list of boundary values. */

  using ElementPositionPair = std::pair<idx, idx>;
  /**
   * @brief Constructs a MeshBase object of the specified type.
   *
   * @param type The type of the mesh.
   */
  explicit TriMesh();

  /**
   * @brief Destructor for the MeshBase object.
   */
  ~TriMesh() = default;

  /**
   * @brief Sets the mesh elements and vertices.
   *
   * @param elements The list of mesh elements.
   * @param vertices The list of mesh vertices.
   * @return The status of the operation.
   */
  void SetMesh(element_list_t const& elements, vertex_list_t const& vertices);

  /**
   * @brief Sets the vertex at the specified index.
   *
   * @param i The index of the vertex.
   * @param vertex The vertex to set.
   */
  void SetVertex(idx i, vertex_t const& vertex);

  /**
   * @brief Sets the list of mesh vertices.
   *
   * @param vertices The list of mesh vertices.
   * @return The status of the operation.
   */
  void SetVertices(vertex_list_t const& vertices);

  /**
   * @brief Gets the vertex at the specified index.
   *
   * @param i The index of the vertex.
   * @return The vertex at the specified index.
   */
  vertex_t GetVertex(idx i) const noexcept;

  /**
   * @brief Gets the list of mesh vertices.
   *
   * @return The list of mesh vertices.
   */
  vertex_list_t const& GetVertices() const noexcept;

  /**
   * @brief Gets the flattened list of mesh vertices.
   *
   * @return The flattened list of mesh vertices.
   */
  math::vecxr GetVerticesFlattened() const noexcept;

  /**
   * @brief Gets the element at the specified index.
   *
   * @param i The index of the element.
   * @return The element at the specified index.
   */
  auto GetElement(idx i) const noexcept;

  /**
   * @brief Gets the list of mesh elements.
   *
   * @return The list of mesh elements.
   */
  element_list_t const& GetElements() const noexcept;

  /**
   * @brief Extracts the surface mesh from the current mesh.
   *
   * @return The surface mesh.
   */
  geo::SurfaceMesh ExtractSurface() const;

  idx GetNumVertices() const noexcept;
  idx GetNumElements() const noexcept;

  /**
   * @brief Resets the boundary at the specified index.
   *
   * @param i The index of the boundary.
   */
  void ResetBoundary(idx i, idx dof);

  /**
   * @brief Marks the boundary at the specified index as a Dirichlet boundary with the given value.
   *
   * @param i The index of the boundary.
   * @param value The value of the Dirichlet boundary.
   */
  void MarkDirichletBoundary(idx i, idx dof, const real& value);

  /**
   * @brief Resets all the boundaries.
   */
  void ResetAllBoundaries();

  /**
   * @brief Gets the boundary value at the specified index.
   *
   * @param i The index of the boundary.
   * @return The boundary value at the specified index.
   */
  real GetBoundaryValue(idx i, idx dof) const noexcept;

  /**
   * @brief Checks if the boundary at the specified index is a Dirichlet boundary.
   *
   * @param i The index of the boundary.
   * @return True if the boundary is a Dirichlet boundary, false otherwise.
   */
  bool IsDirichletBoundary(idx i, idx dof) const noexcept;

  /**
   * @brief Returns an iterator to the beginning of the mesh elements.
   *
   * @return An iterator to the beginning of the mesh elements.
   */
  auto begin() const noexcept;

  /**
   * @brief Returns an iterator to the end of the mesh elements.
   *
   * @return An iterator to the end of the mesh elements.
   */
  AX_FORCE_INLINE auto end() const noexcept;

  /**
   * @brief Returns a constant iterator to the beginning of the mesh elements.
   *
   * @return A constant iterator to the beginning of the mesh elements.
   */
  AX_FORCE_INLINE auto cbegin() const noexcept;

  /**
   * @brief Returns a constant iterator to the end of the mesh elements.
   *
   * @return A constant iterator to the end of the mesh elements.
   */
  AX_FORCE_INLINE auto cend() const noexcept;

  /**
   * @brief Returns an iterator to the beginning of the mesh elements.
   *
   * @return An iterator to the beginning of the mesh elements.
   */
  AX_FORCE_INLINE auto begin() noexcept;

  /**
   * @brief Returns an iterator to the end of the mesh elements.
   *
   * @return An iterator to the end of the mesh elements.
   */
  AX_FORCE_INLINE auto end() noexcept;

  void FilterMatrixFull(math::sp_coeff_list const& input, math::sp_coeff_list& out) const;

  void FilterMatrixFull(math::spmatr& mat) const;

  void FilterMatrixDof(idx dof, math::spmatr& mat) const;

  void FilterMatrixDof(idx dif, math::sp_coeff_list const& input, math::sp_coeff_list& out) const;

  void FilterVector(math::vecxr& inout, bool set_zero = false) const;

  void FilterField(math::fieldr<dim>& inout, bool set_zero = false) const;

  void SetNumDofPerVertex(idx n_dof_per_vertex) noexcept;

  idx GetNumDofPerVertex() const noexcept { return n_dof_per_vertex_; }

  std::vector<std::vector<ElementPositionPair>> const& GetVertexToElementMap() const noexcept {
    return v_e_map_;
  }

  boundary_value_list_t const& GetDirichletBoundaryMask() const noexcept {
    return dirichlet_boundary_mask_;
  }

  void ApplyPermutation(std::vector<idx> const& perm, std::vector<idx> const& inverse_perm);

protected:
  element_list_t elements_; /**< The list of mesh elements. */
  vertex_list_t vertices_;  /**< The list of mesh vertices. */

  // We may need to compute the inverse mapping, i.e. the vertex->elements connected to it.
  std::vector<std::vector<ElementPositionPair>> v_e_map_;

  idx n_dof_per_vertex_;                  /**< The number of degrees of freedom per vertex. */
  boundary_value_list_t boundary_values_; /**< The list of boundary values. */
  boundary_value_list_t dirichlet_boundary_mask_; /**< The mask for Dirichlet boundaries. */
};

// NOTE: Some member functions does not have a FRIENDLY return value, we provide it as
//       a template implementation to avoid the need of defining it in the cpp file.
// NOTE: Also, some member functions should be defined in header file to achieve inlining.
template <idx dim> AX_FORCE_INLINE auto TriMesh<dim>::GetElement(idx i) const noexcept {
  AX_DCHECK(i < elements_.cols()) << "Index out of bounds.";
  return elements_.col(i);
}

template <idx dim>
AX_FORCE_INLINE typename TriMesh<dim>::vertex_t TriMesh<dim>::GetVertex(idx i) const noexcept {
  AX_DCHECK(i < vertices_.cols()) << "Index out of bounds.";
  return vertices_.col(i);
}

template <idx dim> AX_FORCE_INLINE auto TriMesh<dim>::begin() const noexcept {
  return math::each(elements_).begin();
}

template <idx dim> AX_FORCE_INLINE auto TriMesh<dim>::end() const noexcept {
  return math::each(elements_).end();
}

template <idx dim> AX_FORCE_INLINE auto TriMesh<dim>::cbegin() const noexcept {
  return math::each(elements_).cbegin();
}

template <idx dim> AX_FORCE_INLINE auto TriMesh<dim>::cend() const noexcept {
  return math::each(elements_).cend();
}

template <idx dim> AX_FORCE_INLINE auto TriMesh<dim>::begin() noexcept {
  return math::each(elements_).begin();
}

template <idx dim> AX_FORCE_INLINE auto TriMesh<dim>::end() noexcept {
  return math::each(elements_).end();
}

template <idx dim> AX_FORCE_INLINE idx TriMesh<dim>::GetNumVertices() const noexcept {
  return vertices_.cols();
}

template <idx dim> AX_FORCE_INLINE idx TriMesh<dim>::GetNumElements() const noexcept {
  return elements_.cols();
}


template <idx dim>
AX_FORCE_INLINE real TriMesh<dim>::GetBoundaryValue(idx i, idx dof) const noexcept {
  AX_DCHECK(0 <= i && i < boundary_values_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < n_dof_per_vertex_) << "Dof out of range.";
  return boundary_values_(dof, i);
}

template <idx dim>
AX_FORCE_INLINE bool TriMesh<dim>::IsDirichletBoundary(idx i, idx dof) const noexcept {
  AX_DCHECK(0 <= i && i < dirichlet_boundary_mask_.cols()) << "Index out of range.";
  AX_DCHECK(0 <= dof && dof < n_dof_per_vertex_) << "Dof out of range.";
  return dirichlet_boundary_mask_(dof, i) == 0;
}

template <idx dim>
typename TriMesh<dim>::vertex_list_t const& TriMesh<dim>::GetVertices() const noexcept {
  return vertices_;
}

template <idx dim>
typename TriMesh<dim>::element_list_t const& TriMesh<dim>::GetElements() const noexcept {
  return elements_;
}

template <idx dim> void TriMesh<dim>::SetVertex(idx i, vertex_t const& vertex) {
  AX_DCHECK(0 <= i && i < vertices_.size()) << "Index out of range.";
  vertices_.col(i) = vertex;
}

}  // namespace ax::fem
