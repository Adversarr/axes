#pragma once

#include <absl/container/flat_hash_map.h>

#include <typeindex>

namespace axes::gui {

struct UiSysCallbackInfo {
  std::string name_;
  std::function<void(void*)> display_;
};

}  // namespace axes::gui
