#pragma once

#include "axes/geometry/common.hpp"
#include "axes/core/echo.hpp"
#include "axes/math/sparse.hpp"

namespace ax::pde::fem {

AX_DECLARE_ENUM(MeshType){
    kP1,  // P1 Element, e.g. Trig in 2D, and Tet in 3D.
    kQ1,  // Q1 Element, e.g. Quad in 2D, and Hex in 3D.
};

/**
 * @brief A base class for mesh representation in finite element method.
 *
 * This class provides a base implementation for representing a mesh in the finite element method.
 * It defines the data structures and methods necessary for handling the mesh elements, vertices,
 * and boundary conditions.
 *
 * @tparam dim The dimension of the mesh.
 */
template <idx dim> class MeshBase {
public:
  using element_t = math::vecxi; /**< Type for storing mesh elements. */
  using element_list_t = math::fieldxi; /**< Type for storing a list of mesh elements. */

  using vertex_t = math::vecr<dim>; /**< Type for storing mesh vertices. */
  using vertex_list_t = math::fieldr<dim>; /**< Type for storing a list of mesh vertices. */

  using boundary_value_t = vertex_t; /**< Type for storing boundary values. */
  using boundary_value_list_t = vertex_list_t; /**< Type for storing a list of boundary values. */
  using boundary_type_list_t = std::vector<bool>; /**< Type for storing boundary types. */

  /**
   * @brief Constructs a MeshBase object of the specified type.
   *
   * @param type The type of the mesh.
   */
  explicit MeshBase(MeshType type);

  /**
   * @brief Destructor for the MeshBase object.
   */
  virtual ~MeshBase() = default;

  /**
   * @brief Creates a MeshBase object of the specified type.
   *
   * @param type The type of the mesh.
   * @return A unique pointer to the created MeshBase object.
   */
  static std::unique_ptr<MeshBase<dim>> Create(MeshType type);

  /**
   * @brief Gets the type of the mesh.
   *
   * @return The type of the mesh.
   */
  MeshType GetType() const noexcept;

  /**
   * @brief Sets the mesh elements and vertices.
   *
   * @param elements The list of mesh elements.
   * @param vertices The list of mesh vertices.
   * @return The status of the operation.
   */
  Status SetMesh(element_list_t const& elements, vertex_list_t const& vertices);

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
  Status SetVertices(vertex_list_t const& vertices);

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
  virtual geo::SurfaceMesh ExtractSurface() const = 0;

  /**
   * @brief Gets the number of vertices per element.
   *
   * @return The number of vertices per element.
   */
  virtual idx GetNumVerticesPerElement() const noexcept = 0;

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
  bool IsDirichletBoundary(idx i, idx dof ) const noexcept;

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

  void FilterMatrix(math::sp_coeff_list const& input, 
                    math::sp_coeff_list & out) const;
  
  void FilterMatrix(math::sp_matxxr & mat) const;

  void FilterVector(math::vecxr& inout, bool set_zero=false) const;

protected:
  element_list_t elements_; /**< The list of mesh elements. */
  vertex_list_t vertices_; /**< The list of mesh vertices. */
  const MeshType type_; /**< The type of the mesh. */

  boundary_value_list_t boundary_values_; /**< The list of boundary values. */
  math::fieldr<dim> dirichlet_boundary_mask_; /**< The mask for Dirichlet boundaries. */
};

// NOTE: Some member functions does not have a FRIENDLY return value, we provide it as
//       a template implementation to avoid the need of defining it in the cpp file.
// NOTE: Also, some member functions should be defined in header file to achieve inlining.
template <idx dim> AX_FORCE_INLINE auto MeshBase<dim>::GetElement(idx i) const noexcept {
  AX_DCHECK(i < elements_.cols()) << "Index out of bounds.";
  return elements_.col(i);
}

template <idx dim> AX_FORCE_INLINE 
typename MeshBase<dim>::vertex_t MeshBase<dim>::GetVertex(idx i) const noexcept {
  AX_DCHECK(i < vertices_.cols()) << "Index out of bounds.";
  return vertices_.col(i);
}

template <idx dim> AX_FORCE_INLINE auto MeshBase<dim>::begin() const noexcept {
  return math::each(elements_).begin();
}

template <idx dim> AX_FORCE_INLINE auto MeshBase<dim>::end() const noexcept {
  return math::each(elements_).end();
}

template <idx dim> AX_FORCE_INLINE auto MeshBase<dim>::cbegin() const noexcept {
  return math::each(elements_).cbegin();
}

template <idx dim> AX_FORCE_INLINE auto MeshBase<dim>::cend() const noexcept {
  return math::each(elements_).cend();
}

template <idx dim> AX_FORCE_INLINE auto MeshBase<dim>::begin() noexcept {
  return math::each(elements_).begin();
}

template <idx dim> AX_FORCE_INLINE auto MeshBase<dim>::end() noexcept {
  return math::each(elements_).end();
}

template <idx dim> AX_FORCE_INLINE idx MeshBase<dim>::GetNumVertices() const noexcept {
  return vertices_.cols();
}

template <idx dim> AX_FORCE_INLINE idx MeshBase<dim>::GetNumElements() const noexcept {
  return elements_.cols();
}

}  // namespace ax::pde::fem
