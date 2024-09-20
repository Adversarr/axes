#pragma once
#include <gsl/assert>
#include <gsl/pointers>

#define AX_EXPECTS(cond) Expects(cond)
#define AX_ENSURES(cond) Ensures(cond)
#define AX_ASSUME(cond) GSL_ASSUME(cond)

namespace ax {

using gsl::make_not_null;
using gsl::make_strict_not_null;
using gsl::not_null;
using gsl::owner;
using gsl::strict_not_null;

template <typename T>
using shared_not_null = not_null<std::shared_ptr<T>>;

template <typename T>
using unique_not_null = not_null<std::unique_ptr<T>>;

template <typename T>
using weak_not_null = not_null<std::weak_ptr<T>>;

}  // namespace ax