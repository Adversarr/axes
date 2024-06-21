#pragma once
#include <boost/describe/enum.hpp>  // IWYU pragma: export
#include <boost/describe/enum_from_string.hpp>
#include <boost/describe/enum_to_string.hpp>
#include <map>  // IWYU pragma: export
#include <optional>

#include "common.hpp"

/**
 * @namespace ax::utils
 * @brief Contains utility functions for working with enums.
 */
namespace ax::utils {

/**
 * @brief Reflects an enum value from its string representation.
 * @tparam Enum The enum type.
 * @param name The string representation of the enum value.
 * @return An optional containing the reflected enum value, or std::nullopt if the string
 * representation is invalid.
 */
template <typename Enum> std::optional<Enum> reflect_enum(const char* name) {
  using namespace boost::describe;
  if (Enum e; enum_from_string<Enum, describe_enumerators<Enum>>(name, e)) {
    return e;
  } else {
    return std::nullopt;
  }
}
template <typename Enum> std::optional<Enum> reflect_enum(std::string const& name) {
  return reflect_enum<Enum>(name.c_str());
}

/**
 * @brief Retrieves the names of all enum values.
 * @tparam Enum The enum type.
 * @return A vector containing the names of all enum values.
 */
template <typename Enum> List<std::string> reflect_names() {
  List<std::string> names;
  boost::mp11::mp_for_each<boost::describe::describe_enumerators<Enum>>(
      [&](auto D) { names.push_back(D.name()); });
  return names;
}

/**
 * @brief Reflects the string representation of an enum value.
 * @tparam Enum The enum type.
 * @param val The enum value.
 * @return An optional containing the string representation of the enum value, or std::nullopt if
 * the enum value is invalid.
 */
template <typename Enum> std::optional<std::string> reflect_name(Enum val) {
  char buf[256] = {0};  /// Typically a cpp identifier will not exceed 256 char.
  if (boost::describe::enum_to_string(val, buf)) {
    return buf;
  } else {
    return std::nullopt;
  }
}

/**
 * @brief Creates an object of type T based on the enum value's name.
 * @tparam Enum The enum type.
 * @tparam T The object type to create.
 * @param name The name of the enum value.
 * @return A unique pointer to the created object, or nullptr if the enum value is invalid.
 */
template <typename Enum, typename T> UPtr<T> reflect_create(std::string_view name) {
  std::optional<Enum> val = reflect_enum<Enum>(name);
  if (!val.has_value()) {
    return nullptr;
  } else {
    return T::Create(val.name());
  }
}

}  // namespace ax::utils
