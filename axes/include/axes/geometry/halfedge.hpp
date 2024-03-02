#pragma once
#include <vector>

#include "axes/core/config.hpp"
#include "axes/math/common.hpp"
#include "axes/math/fields.hpp"
#include "axes/utils/common.hpp"
#include "axes/utils/iota.hpp"
namespace ax::geo {

struct HalfedgeFace_t;
struct HalfedgeEdge_t;
struct HalfedgeVertex_t;
constexpr idx kHalfedgeInvalidIdx = -1;

struct HalfedgeFace_t {
  HalfedgeEdge_t* halfedge_entry_;
  idx original_idx_ = kHalfedgeInvalidIdx;
};

struct HalfedgeEdge_t {
  HalfedgeEdge_t* next_ = nullptr;
  HalfedgeEdge_t* prev_ = nullptr;
  HalfedgeVertex_t* vertex_ = nullptr;
  HalfedgeFace_t* face_ = nullptr;
  HalfedgeEdge_t* pair_ = nullptr;

  bool IsBoundary() const { return pair_ == nullptr; }
};

struct HalfedgeVertex_t {
  math::vec3r position_;
  idx original_id_ = kHalfedgeInvalidIdx;
  HalfedgeEdge_t* halfedge_entry_ = nullptr;

  HalfedgeVertex_t() = default;

  explicit HalfedgeVertex_t(
    math::vec3r const& position,
    idx original_id = kHalfedgeInvalidIdx)
      : position_(position), original_id_(original_id) {}
};

class HalfedgeMesh {
public:
  HalfedgeMesh() = default;
  AX_DECLARE_COPY_CTOR(HalfedgeMesh, delete);
  AX_DECLARE_MOVE_CTOR(HalfedgeMesh, default);

  HalfedgeMesh(math::field3r const& vertices, math::field3i const& indices);

  [[nodiscard]] std::pair<math::field3r, math::field3i> ToTriangleMesh() const;

  void CollapseEdge(HalfedgeEdge_t* edge, math::vec3r const& target_position);

  enum class CollapseAbility {
    kOk,
    kBoundary,
    kNonManifold,
  };

  bool CheckCollapse(HalfedgeEdge_t* edge);

  HalfedgeEdge_t* TryGetEdgeBetween(HalfedgeVertex_t* v1, HalfedgeVertex_t* v2);

  void ForeachEdgeAroundVertex(HalfedgeVertex_t* vert, std::function<void(HalfedgeEdge_t*)> const& fn);
  void ForeachEdgeInFace(HalfedgeFace_t* face, std::function<void(HalfedgeEdge_t*)> const& fn);

  void ForeachEdge(std::function<void(HalfedgeEdge_t*)> const& fn);

  void ForeachVertex(std::function<void(HalfedgeVertex_t*)> const& fn);


  idx NVertices() const noexcept { return (idx) vertices_.size(); }

  idx NEdges() const noexcept { return (idx) edges_.size(); }

  bool CheckOk() const noexcept;

// private:
  void RemoveVertexInternal(HalfedgeVertex_t* vert);
  void RemoveEdgeInternal(HalfedgeEdge_t* edge);
  void RemoveFaceInternal(HalfedgeFace_t* face);
  std::vector<utils::uptr<HalfedgeVertex_t>> vertices_;
  std::vector<utils::uptr<HalfedgeEdge_t>> edges_;
  std::vector<utils::uptr<HalfedgeFace_t>> faces_;
};

}  // namespace ax::geo