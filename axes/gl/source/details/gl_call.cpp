#include "ax/gl/details/gl_call.hpp"

#include <glad/glad.h>

#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"

namespace ax::gl::details {

std::string to_string(GLenum error_code) {
  switch (error_code) {
    case GL_NO_ERROR:
      return "GL_NO_ERROR";
    case GL_INVALID_ENUM:
      return "GL_INVALID_ENUM";
    case GL_INVALID_VALUE:
      return "GL_INVALID_VALUE";
    case GL_INVALID_OPERATION:
      return "GL_INVALID_OPERATION";
    case GL_INVALID_FRAMEBUFFER_OPERATION:
      return "GL_INVALID_FRAMEBUFFER_OPERATION";
    case GL_OUT_OF_MEMORY:
      return "GL_OUT_OF_MEMORY";
    case GL_STACK_UNDERFLOW:
      return "GL_STACK_UNDERFLOW";
    case GL_STACK_OVERFLOW:
      return "GL_STACK_OVERFLOW";
    default:
      return "UNKNOWN_ERROR";
  }
}

void clear_error() {
  for (GLenum error_code = glGetError(); error_code != GL_NO_ERROR; error_code = glGetError()) {
    AX_ERROR("OpenGL error ignored: {}", to_string(error_code));
  }
}

void fetch_error(const char* expr, const char* file, int line) {
  GLenum error_code = glGetError();
  if (error_code != GL_NO_ERROR) {
    AX_THROW_RUNTIME_ERROR("OpenGL error: {}, \nOccur at {}:{}\ncommand={}", to_string(error_code), file,
                             line, expr);
  }
}

}  // namespace ax::gl::details
