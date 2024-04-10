#pragma once
#include "ax/graph/common.hpp"
#include "ax/graph/pin.hpp"

namespace ax::graph {

struct Socket {
  Socket(idx id, Pin* input, Pin* out) : input_(input), out_(out), id_(id) {}
  Pin* input_;
  Pin* out_;
  idx id_;
};

}  // namespace ax::graph