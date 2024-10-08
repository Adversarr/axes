#pragma once
#include <boost/describe/enum.hpp>
#include <boost/describe/enum_from_string.hpp>
#include <boost/describe/enum_to_string.hpp>
#include <optional>

// Enum should be integer, we rely on the negative value to represent invalid enum.
#define AX_DEFINE_ENUM_CLASS(Enum, ...) BOOST_DEFINE_FIXED_ENUM_CLASS(Enum, int, __VA_ARGS__)

#include "common.hpp" // IWYU pragma: export

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
template <typename Enum>
std::optional<Enum> reflect_enum(const char* name) {
  static_assert(std::is_enum_v<Enum>, "should be a cpp enum.");
  using namespace boost::describe;
  if (Enum e; enum_from_string<Enum, describe_enumerators<Enum>>(name, e)) {
    return e;
  } else {
    return std::nullopt;
  }
}

template <typename Enum>
std::optional<Enum> reflect_enum(std::string const& name) {
  return reflect_enum<Enum>(name.c_str());
}

/**
 * @brief Retrieves the names of all enum values.
 * @tparam Enum The enum type.
 * @return A vector containing the names of all enum values.
 */
template <typename Enum>
std::vector<std::string> reflect_names() {
  static std::vector<std::string> names;
  if (names.empty()) {
    boost::mp11::mp_for_each<boost::describe::describe_enumerators<Enum>>([&](auto D) {
      names.push_back(D.name);
    });
  }
  return names;
}

/**
 * @brief Reflects the string representation of an enum value.
 * @tparam Enum The enum type.
 * @param val The enum value.
 * @return An optional containing the string representation of the enum value, or std::nullopt if
 * the enum value is invalid.
 */
template <typename Enum>
std::optional<std::string> reflect_name(Enum val) {
  if (auto p = boost::describe::enum_to_string(val, nullptr)) {
    return p;
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
template <typename Enum, typename T>
std::unique_ptr<T> reflect_create(std::string_view name) {
  std::optional<Enum> val = reflect_enum<Enum>(name);
  if (!val.has_value()) {
    return nullptr;
  } else {
    return T::Create(val.name());
  }
}

}  // namespace ax::utils
