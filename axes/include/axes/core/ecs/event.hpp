#pragma once
#include "axes/core/common.hpp"
namespace axes::ecs {

enum class EventKind : UInt32 {
  kSystemShutdown,
  kUser,
};

class Event {
public:
  explicit Event(EventKind event_kind, UInt32 flag = 0, void* data = nullptr)
      : event_kind_(event_kind), special_flag_(flag), data_(data) {}

  inline EventKind GetKind() const noexcept { return event_kind_; }

  inline UInt32 GetFlag() const noexcept { return special_flag_; }

  inline void* GetData() const noexcept { return data_; }

private:
  EventKind event_kind_;
  UInt32 special_flag_;
  void* data_;
};
}  // namespace axes::ecs
