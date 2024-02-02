#include "axes/gl/details/gl_call.hpp"

#include <glad/glad.h>

#include "axes/core/echo.hpp"

namespace ax::gl::details {

std::string error_code_to_string(GLenum error_code) {
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
    LOG(WARNING) << "Ignore OpenGL error:" << error_code_to_string(error_code);
  }
}

Status fetch_error() {
  GLenum error_code = glGetError();
  if (error_code == GL_NO_ERROR) {
    return utils::OkStatus();
  }
  return utils::InvalidArgumentError(error_code_to_string(error_code));
}

}  // namespace ax::gl::details
