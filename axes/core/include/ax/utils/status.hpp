#pragma once
#include "ax/core/echo.hpp"    // IWYU pragma: export
#include "ax/core/status.hpp"  // IWYU pragma: export

namespace ax::utils {
/****************************** Checker ******************************/
using absl::IsAborted;
using absl::IsAlreadyExists;
using absl::IsCancelled;
using absl::IsDataLoss;
using absl::IsDeadlineExceeded;
using absl::IsFailedPrecondition;
using absl::IsInternal;
using absl::IsInvalidArgument;
using absl::IsNotFound;
using absl::IsOutOfRange;
using absl::IsPermissionDenied;
using absl::IsResourceExhausted;
using absl::IsUnauthenticated;
using absl::IsUnavailable;
using absl::IsUnimplemented;
using absl::IsUnknown;

/****************************** Ctor ******************************/
using absl::AbortedError;
using absl::AlreadyExistsError;
using absl::CancelledError;
using absl::DataLossError;
using absl::DeadlineExceededError;
using absl::FailedPreconditionError;
using absl::InternalError;
using absl::InvalidArgumentError;
using absl::NotFoundError;
using absl::OkStatus;
using absl::OutOfRangeError;
using absl::PermissionDeniedError;
using absl::ResourceExhaustedError;
using absl::UnauthenticatedError;
using absl::UnavailableError;
using absl::UnimplementedError;
using absl::UnknownError;

#define AX_RETURN_NOTOK(expr)               \
  if (auto status = (expr); !status.ok()) { \
    return status;                          \
  }

#define AX_RETURN_NOTOK_OR(expr)      \
  if (auto sor = (expr); !sor.ok()) { \
    return sor.status();              \
  }

#define AX_EVAL_RETURN_NOTOK(expr)          \
  if (auto status = (expr); !status.ok()) { \
    return status;                          \
  }

#define AX_RETURN_OK() return ::ax::utils::OkStatus()

template <typename T> T&& extract(StatusOr<T>& status_or) { return std::move(status_or.value()); }

template <typename T> T&& extract_or_die(StatusOr<T>& status_or) {
  AX_CHECK_OK(status_or.status());
  return std::move(status_or.value());
}

#define AX_ASSIGN_OR_RETURN(var, expr)     \
  auto var##status = (expr);               \
  AX_RETURN_NOTOK((var##status).status()); \
  auto var = ::ax::utils::extract(var##status)

#define AX_ASSIGN_OR_DIE(var, expr)                                           \
  auto _##var##_status_or = (expr);                                           \
  AX_CHECK_OK((_##var##_status_or).status()) << "Failed preconditon: " #expr; \
  auto var = ::ax::utils::extract_or_die(_##var##_status_or)

}  // namespace ax::utils
