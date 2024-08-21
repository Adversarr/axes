#pragma once

#include <vector>

#include "ax/math/common.hpp"
namespace ax::xpbd {

class ConstraintMap {
public:
  ConstraintMap() = default;
  AX_FORCE_INLINE ConstraintMap(math::IndexMatrixX const& mat) { *this = mat; }
  AX_FORCE_INLINE ConstraintMap& operator=(ConstraintMap const&) = default;
  struct Visitor {
    AX_FORCE_INLINE Visitor(ConstraintMap& map, Index first, Index last)
        : parent_(map), first_(first), last_(last) {}
    AX_FORCE_INLINE Index const& operator[](Index i) const { return parent_.mapping_[first_ + i]; }
    AX_FORCE_INLINE Index& operator[](Index i) { return parent_.mapping_[first_ + i]; }
    AX_FORCE_INLINE Index size() const { return last_ - first_; }
    AX_FORCE_INLINE Index const& at(Index i) const {
      if (i + first_ >= last_) {
        throw std::out_of_range("Visitor out of range");
      }
      return parent_.mapping_.at(first_ + i);
    }
    AX_FORCE_INLINE Index& at(Index i) {
      if (i + first_ >= last_) {
        throw std::out_of_range("Visitor out of range");
      }
      return parent_.mapping_.at(first_ + i);
    }

    ConstraintMap& parent_;
    Index first_, last_;
  };

  template <typename... Args,
            typename = std::enable_if_t<(std::is_integral_v<std::decay_t<Args>> && ...)>>
  AX_FORCE_INLINE Visitor emplace_back(Args&&... args) {
    entries_.push_back(mapping_.size());
    (mapping_.push_back(static_cast<Index>(args)), ...);
    return back();
  }

  template <typename Derived, typename = std::enable_if_t<!std::is_integral_v<Derived>>>
  AX_FORCE_INLINE Visitor emplace_back(math::MBcr<Derived> v) {
    static_assert(Derived::ColsAtCompileTime == 1, "Expect a column vector");
    entries_.push_back(mapping_.size());
    for (Index i = 0; i < v.rows(); ++i) {
      mapping_.push_back(v[i]);
    }
    return back();
  }

  AX_FORCE_INLINE void reserve(Index n_constraint, Index n_vertices_per_constraint) {
    mapping_.reserve(n_constraint * n_vertices_per_constraint);
    entries_.reserve(n_constraint);
  }

  AX_FORCE_INLINE std::vector<size_t> const& Entries() const { return entries_; }
  AX_FORCE_INLINE std::vector<Index> const& Mapping() const { return mapping_; }

  struct ConstVisitor {
    AX_FORCE_INLINE ConstVisitor(ConstraintMap const& map, Index first, Index last)
        : parent_(map), first_(first), last_(last) {}

    AX_FORCE_INLINE ConstVisitor(Visitor const& visitor)
        : parent_(visitor.parent_), first_(visitor.first_), last_(visitor.last_) {}
    AX_FORCE_INLINE Index const& operator[](Index i) const { return parent_.mapping_[first_ + i]; }
    AX_FORCE_INLINE Index size() const { return last_ - first_; }
    AX_FORCE_INLINE Index const& at(Index i) const {
      if (i + first_ >= last_) {
        throw std::out_of_range("Visitor out of range");
      }
      return parent_.mapping_.at(first_ + i);
    }

    ConstraintMap const& parent_;
    Index first_, last_;
  };

  AX_FORCE_INLINE Visitor operator[](size_t i) {
    size_t last = i + 1 < entries_.size() ? entries_[i + 1] : mapping_.size();
    return Visitor(*this, entries_[i], last);
  }

  AX_FORCE_INLINE ConstVisitor operator[](size_t i) const {
    size_t last = i + 1 < entries_.size() ? entries_[i + 1] : mapping_.size();
    return ConstVisitor(*this, entries_[i], last);
  }

  AX_FORCE_INLINE ConstVisitor front() const {
    return ConstVisitor(*this, 0, entries_.empty() ? 0 : entries_[0]);
  }

  AX_FORCE_INLINE ConstVisitor back() const {
    return ConstVisitor(*this, entries_.empty() ? 0 : entries_.size() - 1, mapping_.size());
  }

  AX_FORCE_INLINE Visitor front() {
    return Visitor(*this, 0, entries_.empty() ? 0 : entries_[0]);
  }

  AX_FORCE_INLINE Visitor back() {
    return Visitor(*this, entries_.empty() ? 0 : entries_.size() - 1, mapping_.size());
  }

  AX_FORCE_INLINE bool empty() const { return entries_.empty(); }

  AX_FORCE_INLINE void clear() {
    mapping_.clear();
    entries_.clear();
  }

  AX_FORCE_INLINE ConstraintMap& operator=(math::IndexMatrixX const& mat) {
    reserve(mat.cols(), mat.rows());
    for (Index i = 0; i < mat.cols(); ++i) {
      emplace_back(mat.col(i));
    }
    return *this;
  }

private:
  std::vector<Index> mapping_;
  std::vector<size_t> entries_;

public:
  struct ConstIterator {
    AX_FORCE_INLINE ConstIterator(ConstraintMap const& map, Index i) : map_(map), i_entry_(i) {}

    AX_FORCE_INLINE ConstIterator(ConstIterator const& other)
        : map_(other.map_), i_entry_(other.i_entry_) {}

    AX_FORCE_INLINE bool operator!=(ConstIterator const& other) const {
      return i_entry_ != other.i_entry_;
    }

    AX_FORCE_INLINE bool operator==(ConstIterator const& other) const {
      return i_entry_ == other.i_entry_;
    }

    AX_FORCE_INLINE ConstIterator operator++() {
      ++i_entry_;
      return *this;
    }

    AX_FORCE_INLINE ConstIterator operator++(int) {
      ConstIterator back{*this};
      ++i_entry_;
      return back;
    }

    AX_FORCE_INLINE ConstVisitor operator*() const { return map_[i_entry_]; }

  private:
    ConstraintMap const& map_;
    Index i_entry_;
  };

  struct Iterator {
    AX_FORCE_INLINE Iterator(ConstraintMap& map, size_t i) : map_(map), i_entry_(i) {}

    AX_FORCE_INLINE Iterator(Iterator const& other) : map_(other.map_), i_entry_(other.i_entry_) {}

    AX_FORCE_INLINE bool operator!=(Iterator const& other) const {
      return i_entry_ != other.i_entry_;
    }

    AX_FORCE_INLINE bool operator==(Iterator const& other) const {
      return i_entry_ == other.i_entry_;
    }

    AX_FORCE_INLINE Iterator operator++() {
      ++i_entry_;
      return *this;
    }

    AX_FORCE_INLINE Iterator operator++(int) {
      Iterator back{*this};
      ++i_entry_;
      return back;
    }

    AX_FORCE_INLINE Visitor operator*() { return map_[i_entry_]; }

  private:
    ConstraintMap& map_;
    size_t i_entry_;
  };

  AX_FORCE_INLINE ConstIterator begin() const { return ConstIterator(*this, 0); }
  AX_FORCE_INLINE ConstIterator end() const { return ConstIterator(*this, entries_.size()); }
  AX_FORCE_INLINE Iterator begin() { return Iterator(*this, 0); }
  AX_FORCE_INLINE Iterator end() { return Iterator(*this, entries_.size()); }
};

}  // namespace ax::xpbd
