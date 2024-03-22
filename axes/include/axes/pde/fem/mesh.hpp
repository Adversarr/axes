#pragma once

#include "axes/geometry/common.hpp"

namespace ax::pde::fem {

AX_DECLARE_ENUM(BoundaryType){
    kNone = 0,       ///< The vertex does not have any boundary condition.
    kDirichlet = 1,  ///< The vertex is fixed, and the value is given.
    kNeumann = 2     ///< The vertex is not fixed, but the external force is given.
};

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
  using boundary_type_list_t = std::vector<BoundaryType>; /**< Type for storing boundary types. */

  /**
   * @brief Constructs a MeshBase object of the specified type.
   *
   * @param type The type of the mesh.
   */
  MeshBase(MeshType type);

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
  static utils::uptr<MeshBase<dim>> Create(MeshType type);

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

  /**
   * @brief Resets the boundary at the specified index.
   *
   * @param i The index of the boundary.
   */
  void ResetBoundary(idx i);

  /**
   * @brief Marks the boundary at the specified index as a Dirichlet boundary with the given value.
   *
   * @param i The index of the boundary.
   * @param value The value of the Dirichlet boundary.
   */
  void MarkDirichletBoundary(idx i, const boundary_value_t& value);

  /**
   * @brief Marks the boundary at the specified index as a Neumann boundary with the given value.
   *
   * @param i The index of the boundary.
   * @param value The value of the Neumann boundary.
   */
  void MarkNeumannBoundary(idx i, const boundary_value_t& value);

  /**
   * @brief Resets all the boundaries.
   */
  void ResetAllBoundaries();

  /**
   * @brief Makes a cache for the mesh.
   */
  void MakeCache();

  /**
   * @brief Gets the boundary value at the specified index.
   *
   * @param i The index of the boundary.
   * @return The boundary value at the specified index.
   */
  boundary_value_t GetBoundaryValue(idx i) const noexcept;

  /**
   * @brief Checks if the boundary at the specified index is a Dirichlet boundary.
   *
   * @param i The index of the boundary.
   * @return True if the boundary is a Dirichlet boundary, false otherwise.
   */
  bool IsDirichletBoundary(idx i) const noexcept;

  /**
   * @brief Checks if the boundary at the specified index is a Neumann boundary.
   *
   * @param i The index of the boundary.
   * @return True if the boundary is a Neumann boundary, false otherwise.
   */
  bool IsNeumannBoundary(idx i) const noexcept;

  /**
   * @brief Returns an iterator to the beginning of the mesh elements.
   *
   * @return An iterator to the beginning of the mesh elements.
   */
  AX_FORCE_INLINE auto begin() const noexcept;

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

protected:
  element_list_t elements_; /**< The list of mesh elements. */
  vertex_list_t vertices_; /**< The list of mesh vertices. */
  const MeshType type_; /**< The type of the mesh. */

  boundary_type_list_t boundary_types_; /**< The list of boundary types. */
  boundary_value_list_t boundary_values_; /**< The list of boundary values. */

  math::field1r dirichlet_boundary_mask_; /**< The mask for Dirichlet boundaries. */
};

}  // namespace ax::pde::fem