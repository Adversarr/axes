#pragma once

#include "axes/core/config.hpp" // IWYU pragma: export
#include "axes/core/macros.hpp" // IWYU pragma: export
#include "axes/core/status.hpp" // IWYU pragma: export

namespace ax {
template <typename T> using List=std::vector<T>;
template <typename T> using UPtr=std::unique_ptr<T>;
template <typename T> using Ptr=T*;
template <typename T> using SPtr=std::shared_ptr<T>;
}