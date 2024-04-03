#include "ax/gl/program.hpp"

#include "ax/gl/details/gl_call.hpp"
#include "ax/utils/status.hpp"

namespace ax::gl {

Program::Program() : id_(0) {}

Program& Program::Append(Shader shader) {
  shaders_.push_back(std::move(shader));
  return *this;
}

Program::operator bool() const { return id_ != 0; }

Program::~Program() {
  if (id_) {
    glDeleteProgram(id_);
  }
}

Program::Program(Program&& prog)
    : id_(prog.id_), shaders_(std::move(prog.shaders_)), attrib_locations_(std::move(prog.attrib_locations_)) {
  prog.id_ = 0;
}

Status Program::Link() {
  if (id_) {
    return utils::AlreadyExistsError("Program already linked");
  }
  id_ = glCreateProgram();
  for (auto& shader : shaders_) {
    glAttachShader(id_, shader.GetId());
  }
  glLinkProgram(id_);
  int success;
  glGetProgramiv(id_, GL_LINK_STATUS, &success);
  if (!success) {
    char info_log[512];
    glGetProgramInfoLog(id_, 512, nullptr, info_log);
    return utils::InvalidArgumentError(info_log);
  }
  for (auto& shader : shaders_) {
    glDetachShader(id_, shader.GetId());
  }
  return utils::OkStatus();
}

Status Program::Use() {
  if (id_ == 0) {
    return utils::InvalidArgumentError("Program not linked");
  }
  AXGL_CALLR(glUseProgram(id_));

  AX_RETURN_OK();
}

Status Program::SetUniform(const std::string& name, int value) {
  AXGL_CALLR(glUniform1i(glGetUniformLocation(id_, name.c_str()), value));
  AX_RETURN_OK();
}

Status Program::SetUniform(const std::string& name, float value) {
  AXGL_CALLR(glUniform1f(glGetUniformLocation(id_, name.c_str()), value));
  AX_RETURN_OK();
}

Status Program::SetUniform(const std::string& name, const math::vec2f& value) {
  AXGL_CALLR(glUniform2fv(glGetUniformLocation(id_, name.c_str()), 1, value.data()));
  AX_RETURN_OK();
}

Status Program::SetUniform(const std::string& name, const math::vec3f& value) {
  AXGL_CALLR(glUniform3fv(glGetUniformLocation(id_, name.c_str()), 1, value.data()));
  AX_RETURN_OK();
}

Status Program::SetUniform(const std::string& name, const math::vec4f& value) {
  AXGL_CALLR(glUniform4fv(glGetUniformLocation(id_, name.c_str()), 1, value.data()));
  AX_RETURN_OK();
}

Status Program::SetUniform(const std::string& name, const math::mat2f& value) {
  math::mat2f transpose = value.transpose();
  AXGL_CALLR(glUniformMatrix2fv(glGetUniformLocation(id_, name.c_str()), 1, GL_FALSE, transpose.data()));
  AX_RETURN_OK();
}

Status Program::SetUniform(const std::string& name, const math::mat3f& value) {
  math::mat3f transpose = value.transpose();
  AXGL_CALLR(glUniformMatrix3fv(glGetUniformLocation(id_, name.c_str()), 1, GL_FALSE, transpose.data()));
  AX_RETURN_OK();
}

Status Program::SetUniform(const std::string& name, const math::mat4f& value) {
  math::mat4f transpose = value.transpose();
  AXGL_CALLR(glUniformMatrix4fv(glGetUniformLocation(id_, name.c_str()), 1, GL_FALSE, transpose.data()));
  AX_RETURN_OK();
}

GLuint Program::GetId() const { return id_; }

}  // namespace ax::gl
