#include "gl/shader.hpp"

#include "axes/utils/status.hpp"

namespace ax::gl {

Shader::operator bool() const { return id_ != 0; }

GLuint Shader::GetId() const { return id_; }

ShaderType Shader::GetType() const { return type_; }

StatusOr<Shader> Shader::Compile(const char* source, ShaderType type) {
  GLuint shader_id = glCreateShader(static_cast<GLenum>(type));
  glShaderSource(shader_id, 1, &source, nullptr);
  glCompileShader(shader_id);

  GLint success;
  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &success);
  if (!success) {
    GLint log_length;
    glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &log_length);
    std::vector<GLchar> info_log(log_length);
    glGetShaderInfoLog(shader_id, log_length, nullptr, info_log.data());
    return utils::InvalidArgumentError(info_log.data());
  }

  return Shader(shader_id, type);
}

Shader::Shader(GLuint id, ShaderType shader_type) : id_(id), type_(shader_type) {}

Shader::Shader(Shader&& other) : Shader(other.id_, other.type_) {
  other.id_ = 0;
  other.type_ = ShaderType::kVertex;
}

Shader& Shader::operator=(Shader&& other) {
  this->~Shader();
  id_ = other.id_;
  type_ = other.type_;
  other.id_ = 0;
  other.type_ = ShaderType::kVertex;
  return *this;
}

Shader::~Shader() {
  if (id_) {
    glDeleteShader(id_);
  }
}

Shader Shader::Dummy() { return {0, ShaderType::kVertex}; }

}  // namespace ax::gl
