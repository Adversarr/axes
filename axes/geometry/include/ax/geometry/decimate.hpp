#pragma once
#include "ax/math/sparse.hpp"
#include "ax/geometry/halfedge.hpp"
/**
 * @brief The MeshDecimator class provides functionality to decimate a mesh by reducing the number of vertices.
 */
namespace ax::geo {

/**
 * @brief The MeshDecimator class provides functionality to decimate a mesh by reducing the number of vertices.
 */
class MeshDecimator {
public:
  /**
   * @brief Constructs a MeshDecimator object with the given mesh.
   * @param mesh_ A pointer to the HalfedgeMesh object representing the mesh.
   */
  MeshDecimator(HalfedgeMesh* mesh_);

  /**
   * @brief Sets the target ratio of the decimation.
   * @param ratio The target ratio of the decimation. Must be between 0 and 1.
   * @return A reference to the MeshDecimator object.
   */
  MeshDecimator& SetRatio(real ratio);

  /**
   * @brief Sets the target vertex count of the decimation.
   * @param count The target vertex count of the decimation.
   * @return A reference to the MeshDecimator object.
   */
  MeshDecimator& SetTargetCount(idx count);

  /**
   * @brief Runs the mesh decimation algorithm.
   * @return The status of the decimation process.
   */
  Status Run();

  /**
   * @brief The strategy for edge collapse during decimation.
   */
  enum Strategy {
    kDirect,    /**< Direct strategy: collapses edges directly. */
    kQuadratic,  /**< Quadratic strategy: collapses edges quadratically. */
    kSpectral    /**< Spectral strategy: collapses edges spectrally. */
  };

  /**
   * @brief Sets the strategy for edge collapse during decimation.
   * @param s The strategy for edge collapse.
   * @return A reference to the MeshDecimator object.
   */
  MeshDecimator& SetStrategy(Strategy s);

private:
  Strategy collapse_strategy_ = kDirect; /**< The strategy for edge collapse during decimation. */
  real cost_threshold_{1e9}; /**< The cost threshold for edge collapse. */
  HalfedgeMesh* mesh_; /**< A pointer to the HalfedgeMesh object representing the mesh. */
  idx target_count_; /**< The target vertex count of the decimation. */

  math::matxxr top_k_eigenvecs_;
  math::sp_matxxr laplacian_, projection_matrix_;
  
  math::vecxr mass_, m_tilde_;

  math::matxxr plf_precomputed_, pf_precomputed_;

  real Spectral_RowV(idx original_id);
  real Spectral_RowV(idx collapse_a, idx collapse_b);
  void Precompute_PfPrecomputed();
  void Precompute_PlfPrecomputed();
  void Precompute();
  void DoCollapse(idx a, idx b);

  real EvalCost(idx a, idx b);

  /**
   * @brief Finds the edge to collapse based on the current strategy.
   * @return A pointer to the HalfedgeEdge_t object representing the edge to collapse.
   */
  HalfedgeEdge* FindEdgeToCollapse();
};

}
