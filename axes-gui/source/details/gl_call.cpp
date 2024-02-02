#include "gl/details/gl_call.hpp"

#include <GL/glu.h>
#include <glad/glad.h>

#include "axes/core/echo.hpp"

namespace ax::gl::details {

std::string error_code_to_string(GLenum error_code) {
  const GLubyte *err_str = gluErrorString(error_code);
  return reinterpret_cast<const char *>(err_str);
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
