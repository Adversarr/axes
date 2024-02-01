#pragma once

#include "axes/core/status.hpp"
#include "axes/math/common.hpp"
#include "axes/utils/common.hpp"
#include "shader.hpp"
namespace ax::gui {

class Program {
public:
  using Shader_p = utils::uptr<Shader>;
  Program(Shader_p vertex_shader, Shader_p fragment_shader);
  AX_DECLARE_CONSTRUCTOR(Program, delete, delete);

  operator bool() const;

  Status Link();

  Status Use();

  Status SetUniform(const std::string& name, int value);

  Status SetUniform(const std::string& name, float value);

  Status SetUniform(const std::string& name, const math::vec3f& value);

  Status SetUniform(const std::string& name, const math::mat4f& value);

  Status SetUniform(const std::string& name, void* value_ptr);

  unsigned int GetId() const;

  ~Program();

private:
  Shader_p vertex_shader_;
  Shader_p fragment_shader_;
  unsigned int id_;
};

}  // namespace ax::gui
