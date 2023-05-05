#pragma once
#include "common.hpp"
#include <memory>
#include <vector>

namespace axes {

template <typename C> class ComponentManager {
public:
  struct Component {
    std::unique_ptr<C> data_{nullptr};
    EntityID belonging_;

    inline bool IsValid() const noexcept {
      return data_ == nullptr && belonging_ == InvalidComponentID;
    }

    bool Release() noexcept {
      belonging_ = InvalidComponentID;
      data_ = nullptr;
    }
  };

  /**
   * @brief Query all the entities's ID that obtains component `C`, and insert
   * it into OutputIt
   *
   * @tparam OutputIt output iterator.
   * @param first
   */
  template <typename OutputIt> void QueryAll(OutputIt first) const noexcept {
    for (const Component &inst : instances_) {
      // HACK: Assert the instance is valid.
      // if (inst.IsValid()) [[likely]] {
      *first++ = inst.belonging_;
      // }
    }
  }

  /**
   * @brief Query all the entities that obtains component `C`.
   *
   * @return std::vector<EntityID>
   */
  inline std::vector<EntityID> QueryAll() const noexcept {
    std::vector<EntityID> result;
    result.reserve(instances_.size());
    QueryAll(std::back_inserter(result));
    return result;
  }

private:
  static std::vector<Component> instances_;
};

} // namespace axes