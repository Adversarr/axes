#pragma once
#include <string>

#include "ax/core/config.hpp"
#include "ax/graph/common.hpp"

namespace ax::graph {

struct PinDescriptor {
  TypeIdentifier type_;
  std::string name_;
  std::string description_;
};

struct Pin {
public:
  using desc_ptr = const PinDescriptor*;
  idx const node_id_;
  idx const node_io_index_;
  bool const is_input_;
  desc_ptr const descriptor_;
  idx const id_;
  Payload* payload_ = nullptr;

private:
  friend class Graph;
  idx socket_in_id_ = invalid_id;
  Pin(idx node_id, idx node_io_index, bool is_input, desc_ptr descriptor, idx id,
      Payload* payload = nullptr)
      : node_id_(node_id),
        node_io_index_(node_io_index),
        is_input_(is_input),
        descriptor_(descriptor),
        id_(id),
        payload_(payload) {}
};

}  // namespace ax::graph