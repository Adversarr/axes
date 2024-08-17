#pragma once

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

  explicit operator bool() const;

  void Link();

  void Use();

  /****************************** Uniform Buffer Setters ******************************/
  void SetUniform(const std::string& name, int value);
  void SetUniform(const std::string& name, float value);

  void SetUniform(const std::string& name, const math::FloatVector2& value);
  void SetUniform(const std::string& name, const math::FloatVector3& value);
  void SetUniform(const std::string& name, const math::FloatVector4& value);

  void SetUniform(const std::string& name, const math::FloatMatrix2& value);
  void SetUniform(const std::string& name, const math::FloatMatrix3& value);
  void SetUniform(const std::string& name, const math::FloatMatrix4& value);

  unsigned int GetId() const;

private:
  unsigned int id_;
  std::vector<Shader> shaders_;
  std::vector<ProgramAttribLocation> attrib_locations_;
};

}  // namespace ax::gl
