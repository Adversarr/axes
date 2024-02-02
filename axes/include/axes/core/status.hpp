#pragma once
#include <absl/status/statusor.h>

namespace ax {

/****************************** Status ******************************/
using absl::Status;
using absl::StatusCode;
using absl::StatusOr;
}  // namespace ax

#define AX_RETURN_NOTOK(status) \
  if (!status.ok()) {           \
    return status;              \
  }

#define AX_EVAL_RETURN_NOTOK(expr)               \
  if (auto status = expr; !status.ok()) { \
    return status;                        \
  }

#define AX_RETURN_OK() return ::ax::utils::OkStatus()
