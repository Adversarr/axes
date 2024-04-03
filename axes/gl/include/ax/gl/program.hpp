#pragma once

#include "ax/core/status.hpp"
#include "ax/math/common.hpp"
#include "ax/utils/common.hpp"
#include "shader.hpp"
namespace ax::gl {

struct ProgramAttribLocation {
  int location_;
  std::string name_;
};

class Program {
public:
  Program();
  ~Program();
  AX_DECLARE_COPY_CTOR(Program, delete);
  Program(Program&& prog);

  Program& Append(Shader shader);

  operator bool() const;

  Status Link();

  Status Use();

  /****************************** Uniform Buffer Setters ******************************/
  Status SetUniform(const std::string& name, int value);
  Status SetUniform(const std::string& name, float value);

  Status SetUniform(const std::string& name, const math::vec2f& value);
  Status SetUniform(const std::string& name, const math::vec3f& value);
  Status SetUniform(const std::string& name, const math::vec4f& value);

  Status SetUniform(const std::string& name, const math::mat2f& value);
  Status SetUniform(const std::string& name, const math::mat3f& value);
  Status SetUniform(const std::string& name, const math::mat4f& value);

  unsigned int GetId() const;

private:
  unsigned int id_;
  List<Shader> shaders_;
  List<ProgramAttribLocation> attrib_locations_;
};

}  // namespace ax::gl
