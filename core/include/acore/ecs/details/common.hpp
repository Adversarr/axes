#pragma once
#include <cstddef>
#include <cstdint>
#include <numeric>
namespace axes {

using EntityID = std::ptrdiff_t;
constexpr EntityID InvalidEntity = static_cast<EntityID>(-1);

using ComponentID = std::ptrdiff_t;
constexpr ComponentID InvalidComponentID = static_cast<ComponentID>(-1);
}