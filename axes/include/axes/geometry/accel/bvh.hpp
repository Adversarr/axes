#pragma once

#include "axes/utils/common.hpp"
#include "hittables.hpp"

namespace ax::geo {

constexpr idx invalid_idx = -1;

namespace details {
/****************************** Node Definition ******************************/
template <idx dim> struct BVHNode;
template <idx dim> using BVHNode_p = utils::uptr<BVHNode<dim>>;
template <idx dim> struct BVHNode {
  AlignedBoxN<dim> box_;
  idx id_;
  BVHNode_p<dim> left_, right_;

  bool IsLeaf() const { return id_ != invalid_idx; }

  BVHNode(AlignedBoxN<dim> const& box, idx id = invalid_idx, BVHNode_p<dim> left = nullptr,
          BVHNode_p<dim> right = nullptr)
      : box_(box), id_(id), left_(std::move(left)), right_(std::move(right)) {}

  BVHNode(BVHNode const&) = delete;
};

/****************************** Building Strategy ******************************/

template <idx dim> class BVHBuilder {
public:
  BVHNode_p<dim> Build(std::vector<BVHNode_p<dim>>::iterator begin,
                       std::vector<BVHNode_p<dim>>::iterator end) {
    using namespace math;
    if (begin == end) {
      return nullptr;
    }

    if (std::distance(begin, end) == 1) {
      return std::move(*begin);
    }

    AlignedBoxN<dim> box = (*begin)->box_;
    for (auto it = begin + 1; it != end; ++it) {
      box.extend((*it)->box_);
    }

    vecr<dim> extent = box.max() - box.min();
    idx axis = argmax(extent);

    std::sort(begin, end, [axis](auto const& a, auto const& b) {
      return a->box_.center()[axis] < b->box_.center()[axis];
    });

    auto mid = begin + std::distance(begin, end) / 2;
    auto left = Build(begin, mid);
    auto right = Build(mid, end);
    return std::make_unique<BVHNode<dim>>(box, invalid_idx, left, right);
  }
};
}  // namespace details

template <idx dim> using BVH_BuildStrategy_Basic = details::BVHBuilder<dim>;

/****************************** BVH Definition ******************************/

template <idx dim> class BVH {
public:
  using node_type = details::BVHNode<dim>;
  using node_ptr = details::BVHNode_p<dim>;
  // TODO: Implement

  utils::DoNotUse<BVH> __;

private:
  node_ptr root_;
};

}  // namespace ax::geo
