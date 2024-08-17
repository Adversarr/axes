#pragma once
#include <vector>

#include "ax/core/config.hpp"
#include "ax/math/common.hpp"
#include "ax/utils/common.hpp"
#include "ax/utils/iota.hpp"
#include "common.hpp"
namespace ax::geo {

struct HalfedgeFace;
struct HalfedgeEdge;
struct HalfedgeVertex;

constexpr Index kHalfedgeInvalidIdx = -1;

struct HalfedgeFace {
  HalfedgeEdge* halfedge_entry_;         ///< Any halfedge on the face
  Index original_id_ = kHalfedgeInvalidIdx;  ///< Original id from the input mesh
};

struct HalfedgeVertex {
  math::RealVector3 position_;                      ///< Position of the vertex
  Index original_id_ = kHalfedgeInvalidIdx;     ///< Original id from the input mesh
  HalfedgeEdge* halfedge_entry_ = nullptr;  ///< Any halfedge on the vertex

  HalfedgeVertex() = default;

  explicit HalfedgeVertex(math::RealVector3 const& position, Index original_id = kHalfedgeInvalidIdx)
      : position_(position), original_id_(original_id) {}
};

struct HalfedgeEdge {
  HalfedgeEdge* next_ = nullptr;
  HalfedgeEdge* prev_ = nullptr;
  HalfedgeVertex* vertex_ = nullptr;
  HalfedgeFace* face_ = nullptr;
  HalfedgeEdge* pair_ = nullptr;

  bool IsBoundary() const { return face_ == nullptr; }

  math::RealVector3 Normal() const;

  AX_FORCE_INLINE HalfedgeVertex* Tail() const { return prev_->vertex_; }

  AX_FORCE_INLINE HalfedgeVertex* Head() const { return vertex_; }

  AX_FORCE_INLINE std::pair<HalfedgeVertex*, HalfedgeVertex*> HeadAndTail() const {
    return std::make_pair(Head(), Tail());
  }
};

namespace details {
struct HalfedgeEdgeOnFaceIterator {
  explicit HalfedgeEdgeOnFaceIterator(HalfedgeFace* face)
      : face_(face), current_(face->halfedge_entry_) {}

  using iterator_category = std::forward_iterator_tag;
  using value_type = HalfedgeEdge;
  using difference_type = Index;
  using pointer = HalfedgeEdge*;
  using reference = HalfedgeEdge&;
  using const_reference = HalfedgeEdge const&;
  using const_pointer = HalfedgeEdge const*;

  AX_FORCE_INLINE HalfedgeEdgeOnFaceIterator& operator++() {
    current_ = current_->next_;
    if (current_ == face_->halfedge_entry_) {
      current_ = nullptr;
    }
    return *this;
  }

  AX_FORCE_INLINE value_type& operator*() const { return *current_; }

  AX_FORCE_INLINE pointer operator->() const { return current_; }

  HalfedgeFace* face_;
  HalfedgeEdge* current_;
};

struct HalfedgeEdgeOnVertexIterator {
  explicit HalfedgeEdgeOnVertexIterator(HalfedgeVertex* vertex)
      : vertex_(vertex), current_(vertex->halfedge_entry_) {}

  using iterator_category = std::forward_iterator_tag;
  using value_type = HalfedgeEdge;
  using difference_type = Index;
  using pointer = HalfedgeEdge*;
  using reference = HalfedgeEdge&;
  using const_reference = HalfedgeEdge const&;
  using const_pointer = HalfedgeEdge const*;

  AX_FORCE_INLINE HalfedgeEdgeOnVertexIterator& operator++() {
    current_ = current_->pair_->next_;
    if (current_ == vertex_->halfedge_entry_) {
      current_ = nullptr;
    }
    return *this;
  }

  AX_FORCE_INLINE value_type& operator*() const { return *current_; }

  AX_FORCE_INLINE pointer operator->() const { return current_; }

  AX_FORCE_INLINE bool operator==(HalfedgeEdgeOnVertexIterator const& other) const {
    return current_ == other.current_;
  }

  AX_FORCE_INLINE bool operator!=(HalfedgeEdgeOnVertexIterator const& other) const {
    return current_ != other.current_;
  }

  HalfedgeVertex* vertex_;
  HalfedgeEdge* current_;
};
}  // namespace details

struct HalfedgeVertexHandle {
  HalfedgeVertex* vertex_;
  explicit HalfedgeVertexHandle(HalfedgeVertex* vertex) : vertex_(vertex) {}

  AX_FORCE_INLINE details::HalfedgeEdgeOnVertexIterator begin() const {
    return details::HalfedgeEdgeOnVertexIterator(vertex_);
  }

  AX_FORCE_INLINE details::HalfedgeEdgeOnVertexIterator end() const {
    return details::HalfedgeEdgeOnVertexIterator(nullptr);
  }

  AX_FORCE_INLINE explicit operator bool() const { return vertex_ != nullptr; }

  AX_FORCE_INLINE HalfedgeVertex* operator->() const { return vertex_; }

  AX_FORCE_INLINE HalfedgeVertex& operator*() const { return *vertex_; }
};

struct HalfedgeFaceHandle {
  HalfedgeFace* face_;
  explicit HalfedgeFaceHandle(HalfedgeFace* face) : face_(face) {}

  AX_FORCE_INLINE details::HalfedgeEdgeOnFaceIterator begin() const {
    return details::HalfedgeEdgeOnFaceIterator(face_);
  }

  AX_FORCE_INLINE details::HalfedgeEdgeOnFaceIterator end() const {
    return details::HalfedgeEdgeOnFaceIterator(nullptr);
  }

  AX_FORCE_INLINE explicit operator bool() const { return face_ != nullptr; }

  AX_FORCE_INLINE HalfedgeFace* operator->() const { return face_; }

  AX_FORCE_INLINE HalfedgeFace& operator*() const { return *face_; }
};

class HalfedgeMesh {
public:
  HalfedgeMesh() = default;
  AX_DECLARE_COPY_CTOR(HalfedgeMesh, delete);
  AX_DECLARE_MOVE_CTOR(HalfedgeMesh, default);
  /************************* SECT: Constructor and IO *************************/
  HalfedgeMesh(math::RealField3 const& vertices, math::IndexField3 const& indices);

  AX_NODISCARD SurfaceMesh ToTriangleMesh() const;

  /************************* SECT: Getters *************************/
  AX_NODISCARD HalfedgeFaceHandle GetFace(Index Index) const;
  AX_NODISCARD HalfedgeVertexHandle GetVertex(Index Index) const;

  /************************* SECT: Edge Operation *************************/
  void CollapseEdge(HalfedgeEdge* edge, math::RealVector3 const& target_position);
  AX_NODISCARD bool CheckCollapse(HalfedgeEdge* edge);
  AX_NODISCARD bool CheckFlip(HalfedgeEdge* edge);
  void FlipEdge(HalfedgeEdge* edge);

  HalfedgeEdge* TryGetEdgeBetween(HalfedgeVertex* v1, HalfedgeVertex* v2);

  // Visitors
  void ForeachEdgeAroundVertex(HalfedgeVertex* vert,
                               std::function<void(HalfedgeEdge*)> const& fn);
  void ForeachEdgeInFace(HalfedgeFace* face, std::function<void(HalfedgeEdge*)> const& fn);
  void ForeachEdge(std::function<void(HalfedgeEdge*)> const& fn);
  void ForeachVertex(std::function<void(HalfedgeVertex*)> const& fn);


  /************************* SECT: Metadata Retrival *************************/
  AX_NODISCARD Index NVertices() const noexcept { return static_cast<Index>(vertices_.size()); }
  AX_NODISCARD Index NEdges() const noexcept { return static_cast<Index>(edges_.size()); }
  AX_NODISCARD bool CheckOk() noexcept;


private:
  void RemoveVertexInternal(HalfedgeVertex* vert);
  void RemoveEdgeInternal(HalfedgeEdge* edge);
  void RemoveFaceInternal(HalfedgeFace* face);
  std::vector<std::unique_ptr<HalfedgeVertex>> vertices_;
  std::vector<std::unique_ptr<HalfedgeEdge>> edges_;
  std::vector<std::unique_ptr<HalfedgeFace>> faces_;
};

inline math::RealVector3 HalfedgeEdge::Normal() const {
  if (face_ == nullptr) {
    if (pair_->face_ == nullptr) {
      return math::RealVector3::Zero();
    }
    return pair_->Normal();
  }
  auto const& A = vertex_->position_;
  auto const& B = next_->vertex_->position_;
  auto const& C = next_->next_->vertex_->position_;
  return (B - A).cross(C - A);
}

}  // namespace ax::geo
