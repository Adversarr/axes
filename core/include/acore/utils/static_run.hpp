#pragma once

namespace axes::utils {

template <typename Derived> struct StaticRunner {
  StaticRunner() { static_cast<const Derived*>(this)->Run(); }
};

}  // namespace axes::utils
