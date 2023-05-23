#pragma once
#include <array>
#include <tuple>
#include <type_traits>
#include <utility>
namespace axes::utils::details {
template <typename T, typename... Args>
std::array<T, sizeof...(Args)> make_stl_array(Args&&... args) {
  return {static_cast<T>(args)...};
}

}  // namespace axes::utils::details
