#include "ax/gl/program.hpp"

#include "ax/core/excepts.hpp"
#include "ax/gl/details/gl_call.hpp"

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
    : id_(prog.id_),
      shaders_(std::move(prog.shaders_)),
      attrib_locations_(std::move(prog.attrib_locations_)) {
  prog.id_ = 0;
}

void Program::Link() {
  if (id_) {
    throw std::runtime_error("Program already linked");
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
    throw make_runtime_error("Program link error: {}", info_log);
  }
  for (auto& shader : shaders_) {
    glDetachShader(id_, shader.GetId());
  }
}

void Program::Use() {
  if (id_ == 0) {
    throw std::runtime_error("Program not linked");
  }
  AXGL_CALL(glUseProgram(id_));
}

void Program::SetUniform(const std::string& name, int value) {
  AXGL_CALL(glUniform1i(glGetUniformLocation(id_, name.c_str()), value));
}

void Program::SetUniform(const std::string& name, float value) {
  AXGL_CALL(glUniform1f(glGetUniformLocation(id_, name.c_str()), value));
}

void Program::SetUniform(const std::string& name, const math::FloatVector2& value) {
  AXGL_CALL(glUniform2fv(glGetUniformLocation(id_, name.c_str()), 1, value.data()));
}

void Program::SetUniform(const std::string& name, const math::FloatVector3& value) {
  AXGL_CALL(glUniform3fv(glGetUniformLocation(id_, name.c_str()), 1, value.data()));
}

void Program::SetUniform(const std::string& name, const math::FloatVector4& value) {
  AXGL_CALL(glUniform4fv(glGetUniformLocation(id_, name.c_str()), 1, value.data()));
}

void Program::SetUniform(const std::string& name, const math::FloatMatrix2& value) {
  math::FloatMatrix2 transpose = value.transpose();
  AXGL_CALL(
      glUniformMatrix2fv(glGetUniformLocation(id_, name.c_str()), 1, GL_FALSE, transpose.data()));
}

void Program::SetUniform(const std::string& name, const math::FloatMatrix3& value) {
  math::FloatMatrix3 transpose = value.transpose();
  AXGL_CALL(
      glUniformMatrix3fv(glGetUniformLocation(id_, name.c_str()), 1, GL_FALSE, transpose.data()));
}

void Program::SetUniform(const std::string& name, const math::FloatMatrix4& value) {
  math::FloatMatrix4 transpose = value.transpose();
  AXGL_CALL(
      glUniformMatrix4fv(glGetUniformLocation(id_, name.c_str()), 1, GL_FALSE, transpose.data()));
}

GLuint Program::GetId() const { return id_; }

}  // namespace ax::gl
