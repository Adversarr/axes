#pragma once
#include <string>

#include "ax/core/config.hpp"
#include "ax/graph/common.hpp"

namespace ax::graph {

struct PinDescriptor {
  type_index type_;          ///< The type of the pin.
  std::string name_;         ///< The name of the pin.
  std::string description_;  ///< The description of the pin.
};

template <typename T>
inline PinDescriptor make_pin_descriptor(std::string name, std::string description) noexcept {
  return PinDescriptor{typeid(T), std::move(name), std::move(description)};
}

struct Pin {
public:
  using desc_ptr = const PinDescriptor*;
  ident_t const node_id_;
  ident_t const node_io_index_;
  bool const is_input_;
  desc_ptr const descriptor_;
  ident_t const id_;
  Payload* payload_ = nullptr;

private:
  friend class Graph;
  ident_t socket_in_id_ = INVALID_ID;

  // The only constructor.
  Pin(ident_t node_id, ident_t node_io_index, bool is_input, desc_ptr descriptor, ident_t id,
      Payload* payload = nullptr)
      : node_id_(node_id),
        node_io_index_(node_io_index),
        is_input_(is_input),
        descriptor_(descriptor),
        id_(id),
        payload_(payload) {}
};

}  // namespace ax::graph
