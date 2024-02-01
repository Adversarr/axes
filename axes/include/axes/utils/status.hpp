#pragma once
#include "axes/core/status.hpp" // IWYU pragma: export

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
using absl::OutOfRangeError;
using absl::PermissionDeniedError;
using absl::ResourceExhaustedError;
using absl::UnauthenticatedError;
using absl::UnavailableError;
using absl::UnimplementedError;
using absl::UnknownError;
using absl::OkStatus;
}  // namespace ax::utils
