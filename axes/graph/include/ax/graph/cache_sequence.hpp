#pragma once
#include "ax/core/config.hpp"
namespace ax::graph {

struct CacheSequenceUpdateEvent {
  Index required_frame_id_;
  bool is_cleanup_;
};

}