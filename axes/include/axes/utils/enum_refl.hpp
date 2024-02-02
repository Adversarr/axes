#pragma once
#include <entt/meta/factory.hpp>
#include <entt/meta/meta.hpp>
#include <entt/meta/resolve.hpp>
#include <optional>
#include <string>

namespace ax::utils {

namespace details {
template <typename Enum> struct EnumReflectorMetaBuild {};

}  // namespace details

template <typename Enum> std::optional<Enum> reflect_enum(std::string_view name) {
  static details::EnumReflectorMetaBuild<Enum> setup_reflect;
  Enum* data = entt::resolve<Enum>()
                   .data(entt::hashed_string{name.data()})
                   .get({})
                   .template try_cast<Enum>();

  if (data == nullptr) {
    return std::nullopt;
  } else {
    return *data;
  }
}
}  // namespace ax::utils

#ifndef AX_ENUM_REFL_BEGIN
#  define AX_ENUM_REFL_BEGIN(Enum)                                          \
    template <> struct ax::utils::details::EnumReflectorMetaBuild<Enum> { \
      using enum_type = Enum;                                               \
      EnumReflectorMetaBuild() {                                            \
        entt::meta<enum_type>()
#  define AX_ENUM_STATE(E, N) .data<enum_type ::E>(entt ::hashed_string{#N})
# define AX_ENUM_STATEk(E) AX_ENUM_STATE(k##E, E)

#  define AX_ENUM_REFL_END() \
    ;                        \
    }                        \
    }

#endif
