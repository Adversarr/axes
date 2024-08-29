#pragma once
#include <string>
#include <string_view>
#include <glad/glad.h>

#include "ax/utils/common.hpp"
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
  explicit operator bool() const;

  GLuint GetId() const;

  ShaderType GetType() const;

  static Shader CompileSource(const char* source, ShaderType type);
  static Shader CompileFile(std::string_view file_path, ShaderType type);

  /****************************** Internal Vars ******************************/
private:
  GLuint id_{0};
  ShaderType type_{ShaderType::kVertex};
};

}  // namespace ax::gl
