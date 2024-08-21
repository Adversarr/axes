#pragma once
#include "ax/graph/common.hpp"

namespace ax::nodes {

void register_stl_types(graph::NodeRegistry& reg);

void register_math_types(graph::NodeRegistry& reg);

void register_io(graph::NodeRegistry& reg);

void register_gl(graph::NodeRegistry& reg);

}