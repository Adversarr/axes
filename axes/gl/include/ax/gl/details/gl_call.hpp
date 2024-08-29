#include <string>
#include <glad/glad.h>  // IWYU pragma: export
#include "ax/utils/common.hpp"  // IWYU pragma: export
namespace ax::gl::details {

std::string to_string(GLenum error_code);

/**
 * @brief Clears all OpenGL errors. Report any errors to the log.
 */
void clear_error();

/**
 * @brief Fetches the error code from OpenGL and throws an exception
 *        if there is an error.
 */
void fetch_error(const char* expr, const char* file, int line);
}  // namespace ax::gl::details

// This will check for errors and return the error status if there is an error
#define AXGL_CALL(expr)                                        \
  do {                                                         \
    ::ax::gl::details::clear_error();                          \
    expr;                                                      \
    ::ax::gl::details::fetch_error(#expr, __FILE__, __LINE__); \
  } while (false)
