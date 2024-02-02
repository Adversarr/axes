#include "axes/gl/program.hpp"

#include "axes/utils/status.hpp"
#include "axes/gl/details/gl_call.hpp"

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


}  // namespace ax::gl
