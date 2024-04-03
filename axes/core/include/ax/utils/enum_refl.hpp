#pragma once
#include <map>  // IWYU pragma: export
#include <optional>

#include "common.hpp"

/**
 * @namespace ax::utils
 * @brief Contains utility functions for working with enums.
 */
namespace ax::utils {

namespace details {

/**
 * @brief Meta builder for enum reflection.
 * @tparam Enum The enum type.
 */
template <typename Enum> struct EnumReflectorMetaBuild {};

/**
 * @brief Retrieves the meta information for the given enum type.
 * @tparam Enum The enum type.
 * @return The meta information for the enum type.
 */
template <typename Enum> EnumReflectorMetaBuild<Enum> const& get_meta() {
  static EnumReflectorMetaBuild<Enum> meta;
  return meta;
}

}  // namespace details

/**
 * @brief Reflects an enum value from its string representation.
 * @tparam Enum The enum type.
 * @param name The string representation of the enum value.
 * @return An optional containing the reflected enum value, or std::nullopt if the string representation is invalid.
 */
template <typename Enum> std::optional<Enum> reflect_enum(std::string const& name) {
  auto const& string_to_enum = details::get_meta<Enum>().string_to_enum_;
  if (auto it = string_to_enum.find(name); it != string_to_enum.end()) {
    return it->second;
  } else {
    return std::nullopt;
  }
}

/**
 * @brief Retrieves the names of all enum values.
 * @tparam Enum The enum type.
 * @return A vector containing the names of all enum values.
 */
template <typename Enum> List<std::string> const& reflect_names() {
  return details::get_meta<Enum>().names_;
}

/**
 * @brief Reflects the string representation of an enum value.
 * @tparam Enum The enum type.
 * @param val The enum value.
 * @return An optional containing the string representation of the enum value, or std::nullopt if the enum value is invalid.
 */
template <typename Enum> std::optional<std::string> reflect_name(Enum val) {
  auto const& enum_to_string = details::get_meta<Enum>().enum_to_string_;
  if (auto it = enum_to_string.find(val); it != enum_to_string.end()) {
    return it->second;
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

#ifndef AX_ENUM_REFL_BEGIN
#  define AX_ENUM_REFL_BEGIN(Enum)                                        \
    template <> struct ax::utils::details::EnumReflectorMetaBuild<Enum> { \
      using enum_type = Enum;                                             \
      EnumReflectorMetaBuild() {
#  define AX_ENUM_STATE(E, N)                  \
    enum_to_string_.emplace(enum_type::E, #N); \
    string_to_enum_.emplace(#N, enum_type::E); \
    names_.push_back(#N);
#  define AX_ENUM_STATEk(E) AX_ENUM_STATE(k##E, E)

#  define AX_ENUM_REFL_END()                          \
    }                                                 \
    std::map<enum_type, std::string> enum_to_string_; \
    std::map<std::string, enum_type> string_to_enum_; \
    List<std::string> names_;                  \
    }

#endif
