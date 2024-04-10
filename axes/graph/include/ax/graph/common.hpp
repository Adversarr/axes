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

constexpr idx invalid_id = -3407;
}  // namespace ax::graph
