#include "ax/geometry/accel/flat_octree.hpp"
#include "ax/utils/ndrange.hpp"

#include <range/v3/view/enumerate.hpp>

namespace ax::geo {

// TreeNode make_new_tree_node(std::vector)
class TreeNode;

using it3 = utils::DupTuple<Index, 3>;

struct BroadPhase_FlatOctree::Impl {
  Impl() = default;
  void BuildTree();

  Real root_level_size_{1.0};
  Index maximum_depth_{5};
  std::map<it3, TreeNode> nodes_;
  std::vector<Index> large_colliders_;
  std::vector<ColliderInfo> colliders_;
};

std::optional<it3> get_holding_root(Real size, AlignedBox3 const& aabb) {
  Index ix = static_cast<Index>(floor(aabb.min().x() / size));
  Index iy = static_cast<Index>(floor(aabb.min().y() / size));
  Index iz = static_cast<Index>(floor(aabb.min().z() / size));
  Index jx = static_cast<Index>(floor(aabb.max().x() / size));
  Index jy = static_cast<Index>(floor(aabb.max().y() / size));
  Index jz = static_cast<Index>(floor(aabb.max().z() / size));
  if (ix == jx && iy == jy && iz == jz) {
    return it3(ix, iy, iz);
  } else {
    return std::nullopt;
  }
}


AlignedBox3 get_subnode_aabb(AlignedBox3 const& node, Index i, Index j, Index k) {
  math::RealVector3 size = node.sizes() * 0.5;
  return AlignedBox3{
      node.min() + math::RealVector3{size.x() * i, size.y() * j, size.z() * k},
      node.min() + math::RealVector3{size.x() * (i + 1), size.y() * (j + 1), size.z() * (k + 1)}};
}

std::optional<Index> try_get_subnode(AlignedBox3 const& node, AlignedBox3 const& aabb) {
  for (auto [i, j, k]: utils::ndrange<Index>(2, 2, 2)) {
    auto subnode = get_subnode_aabb(node, i, j, k);
    if (subnode.contains(aabb)) return i * 4 + j * 2 + k;
  }
  return std::nullopt;
}


class TreeNode {
public:
  TreeNode(AlignedBox3 const& aabb, TreeNode const* parent, BroadPhase_FlatOctree::Impl const* impl)
      : aabb_(aabb),
        parent_(parent),
        depth_(parent == nullptr ? 0 : parent->depth_ + 1),
        impl_(impl) {}

  AX_FORCE_INLINE bool Intersect(AlignedBox3 const& aabb) const { return aabb.intersects(aabb_); }
  AX_FORCE_INLINE bool Contain(AlignedBox3 const& aabb) const { return aabb_.contains(aabb); }

  void Build() {
    auto get_collider = [this](Index i) -> ColliderInfo const& { return impl_->colliders_[i]; };
    if (colliders_.empty()) return;
    if (depth_ >= impl_->maximum_depth_) return;
    // ensure all the childrens
    for (auto [i, j, k]: utils::ndrange<Index>(2, 2, 2)) {
      auto& child = children_[i * 4 + j * 2 + k];
      auto aabb = get_subnode_aabb(aabb_, i, j, k);
      if (!child) child = std::make_unique<TreeNode>(aabb, this, impl_);

      assert(Contain(child->aabb_) && "Child AABB does not contained.");
    }
    // Foreach collider:
    std::vector<Index> reserved_by_this;
    for (Index i: colliders_) {
      auto const& collider = get_collider(i);
      bool has_put_to_child = false;
      for (auto const& child: children_) {
        if (child->Contain(collider.aabb_)) {
          child->colliders_.push_back(i);
          has_put_to_child = true;
          break;
        }
      }

      if (!has_put_to_child) {
        reserved_by_this.push_back(i);
      }
    }
    for (auto& child: children_) {
      child->Build();
    }

    colliders_ = std::move(reserved_by_this);
  }

  void DetectCollisions(std::vector<Index> const& id, std::vector<std::pair<Index, Index>> & ret) const {
    auto get_collider = [this](Index i) -> ColliderInfo const& { return impl_->colliders_[i]; };
    std::vector<Index> real_collision;
    for (auto const& i: id) {
      auto const& a = get_collider(i);
      if (! Intersect(a.aabb_)) {
        continue;
      }
      real_collision.push_back(i);
    }

    for (auto const& j: real_collision) {
      auto const& b = get_collider(j);
      if (! Intersect(b.aabb_)) {
        continue;
      }

      for (auto i: colliders_) {
        if (i == j) continue;
        auto const& a = get_collider(i);
        if (a.aabb_.intersects(b.aabb_)) {
          ret.push_back({i, j});
        }
      }
    }

    for (auto const& child: children_) {
      if (child) child->DetectCollisions(real_collision, ret);
    }
  }

  void DetectSelfCollisions(std::vector<std::pair<Index, Index>> & ret) const {
    DetectCollisions(colliders_, ret);
    for (auto const& child: children_) {
      if (child) child->DetectSelfCollisions(ret);
    }
  }

  AlignedBox3 aabb_;
  TreeNode const* parent_;
  Index const depth_;
  BroadPhase_FlatOctree::Impl const* impl_;
  std::array<std::unique_ptr<TreeNode>, 8> children_;  ///< child nodes
  std::vector<Index> colliders_;              ///< for those collider cannot be divided further

  TreeNode(TreeNode&& other) : aabb_(other.aabb_), parent_(other.parent_), depth_(other.depth_), impl_(other.impl_) {
    for (Index i = 0; i < 8; ++i) {
      children_[i] = std::move(other.children_[i]);
    }
    colliders_ = std::move(other.colliders_);
  }
};

BroadPhase_FlatOctree::BroadPhase_FlatOctree() : impl_(std::make_unique<Impl>()) {}

BroadPhase_FlatOctree::~BroadPhase_FlatOctree() {}

void BroadPhase_FlatOctree::DetectCollisions() {
  // 1. Clear previous results
  ClearCollidingPairs();
  impl_ = std::make_unique<Impl>();
  impl_->colliders_ = colliders_;
  impl_->BuildTree();

  // 2. build result
  std::vector<std::pair<Index, Index>> ret;
  for (auto const& [i, ai]: utils::views::enumerate(impl_->large_colliders_)) {
    for (auto const& [j, aj]: utils::views::enumerate(impl_->large_colliders_)) {
      if (i >= j) continue;
      if (colliders_[ai].aabb_.intersects(colliders_[aj].aabb_)) {
        ret.push_back({ai, aj});
      }
    }
  }
  for (auto const& [_, tree]: impl_->nodes_) {
    tree.DetectSelfCollisions(ret);
    tree.DetectCollisions(impl_->large_colliders_, ret);
  }
  for (auto & pair: ret) {
    if (pair.first > pair.second) std::swap(pair.first, pair.second);
  }
  std::sort(ret.begin(), ret.end());
  size_t actual = 0;
  for (size_t i = 0; i < ret.size(); ++i) {
    if (i == 0 || (ret[i] != ret[i - 1])) {
      ret[actual++] = ret[i];
    }
  }
  ret.resize(actual);
  for (auto const& [a, b]: ret) {
    AddCollidingPair(a, b);
  }
}



void BroadPhase_FlatOctree::Impl::BuildTree() {
  // 1. Calculate the root level size
  for (auto [i, collider]: utils::views::enumerate(colliders_)) {
    auto holding = get_holding_root(root_level_size_, collider.aabb_);
    if (holding) {
      if (nodes_.find(*holding) == nodes_.end()) {
        auto [x, y, z] = holding.value();
        AlignedBox3 node_aabb{
          math::IndexVector3{x, y, z}.cast<Real>() * root_level_size_,
          math::IndexVector3{x+1, y+1, z+1}.cast<Real>() * root_level_size_,
        };
        nodes_.emplace(*holding, TreeNode{node_aabb, nullptr, this});
      }
      nodes_.at(*holding).colliders_.push_back(i);
    } else {
      large_colliders_.push_back(i);
    }
  }

  for (auto& [_, tree]: nodes_) {
    tree.Build();
  }
}
template<typename F>
void do_fn_on_node(TreeNode const& node, F f) {
  f(node.aabb_);
  for (auto const& child: node.children_) {
    if (child) do_fn_on_node(*child, f);
  }
}

void BroadPhase_FlatOctree::ForeachTreeAABB(std::function<void(AlignedBox3 const&)> f) const {
  for (auto const& [_, tree]: impl_->nodes_) {
    do_fn_on_node(tree, f);
  }
}

}
