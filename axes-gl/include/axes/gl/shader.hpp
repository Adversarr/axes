#pragma once
#include <glad/glad.h>

#include "axes/core/status.hpp"
#include "axes/utils/common.hpp"
#include "details/gl_types.hpp"

namespace ax::gl {

class Shader {
public:
  /****************************** Ctor and Dtors ******************************/
  AX_DECLARE_COPY_CTOR(Shader, delete);
  Shader(Shader&&);
  Shader& operator=(Shader&&);
  ~Shader();

private:
  Shader(GLuint id, ShaderType shader_type);

  /****************************** Other methods ******************************/
public:
  Shader();
  // Validate the Shader
  operator bool() const;

  GLuint GetId() const;

  ShaderType GetType() const;

  static StatusOr<Shader> Compile(const char* source, ShaderType type);


  /****************************** Internal Vars ******************************/
private:
  GLuint id_{0};
  ShaderType type_{ShaderType::kVertex};
};

}  // namespace ax::gl
