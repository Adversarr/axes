#pragma once
namespace ax::nodes {

namespace details {

void register_stl_types();

struct AutoRegisterStlTypes {
  AutoRegisterStlTypes() { register_stl_types(); }
};
inline static AutoRegisterStlTypes instance_;
}  // namespace details
}  // namespace ax::nodes