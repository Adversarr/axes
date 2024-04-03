#include <glad/glad.h>  // IWYU pragma: export

#include "ax/core/echo.hpp"
#include "ax/utils/common.hpp"  // IWYU pragma: export
#include "ax/utils/status.hpp"

namespace ax::gl::details {

void clear_error();

Status fetch_error();

}  // namespace ax::gl::details
#ifdef AXGL_NO_AX_CHECK
#  define AXGL_AX_CHECK_OK(status) AX_UNUSED(status)
#  define AXGL_AX_CHECK_NOTOK(status) AX_UNUSED(status)
#else
#  define AXGL_AX_CHECK_OK(status, expr) AX_CHECK_OK(status) << "Failed: " << #expr
#  define AXGL_EVAL_RETURN_NOTOK(expr) AX_EVAL_RETURN_NOTOK(expr)
#endif

#ifdef AXGL_NO_AX_CHECK
#  define AXGL_CALL(expr) expr
#else
#define AXGL_CALL(expr)           \
  ax::gl::details::clear_error(); \
  expr
#endif

// This will check for errors and return the error status if there is an error
#define AXGL_CALLR(expr) \
  AXGL_CALL(expr);       \
  AXGL_EVAL_RETURN_NOTOK(ax::gl::details::fetch_error())

// This will check for errors and abort the program if there is an error
#define AXGL_CALLC(expr) \
  AXGL_CALL(expr);       \
  AXGL_AX_CHECK_OK(ax::gl::details::fetch_error(), expr)
