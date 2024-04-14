#pragma once

#include "ax/core/config.hpp"
#include <typeindex>
namespace ax::graph {

using TypeIdentifier = std::type_index;

struct PinDescriptor;
struct NodeDescriptor;

class Payload;
struct Pin;
struct Socket;
class NodeBase;

constexpr idx INVALID_ID = -3407;

struct Socket {
  Socket(idx id, Pin* input, Pin* out) : input_(input), output_(out), id_(id) {}
  Pin* input_;
  Pin* output_;
  idx id_;
};

}  // namespace ax::graph
