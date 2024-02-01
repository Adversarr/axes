#pragma once
#include <glad/glad.h>

#include "axes/core/status.hpp"
#include "axes/utils/common.hpp"
#include "axes/utils/enum_refl.hpp"

namespace ax::gui {

enum class ShaderType {
  kVertex = GL_VERTEX_SHADER,
  kFragment = GL_FRAGMENT_SHADER,
  kGeometry = GL_GEOMETRY_SHADER,
  kTessControl = GL_TESS_CONTROL_SHADER,
  kTessEvaluation = GL_TESS_EVALUATION_SHADER,
  kCompute = GL_COMPUTE_SHADER,
};

class Shader {
public:
  AX_DECLARE_COPY_CTOR(Shader, delete);

  Shader(Shader &&);
  Shader& operator=(Shader &&);


  ~Shader();

  operator bool() const;

  GLuint GetId() const;

  ShaderType GetType() const;

  static StatusOr<Shader> Compile(const char* source, ShaderType type);

  static Shader Dummy();

private:
  Shader(GLuint id, ShaderType shader_type);
  GLuint id_{0};
  ShaderType type_{ShaderType::kVertex};
};

}  // namespace ax::gui

AX_ENUM_REFL_BEGIN(ax::gui::ShaderType)
AX_ENUM_STATEk(Vertex) AX_ENUM_STATEk(Fragment) AX_ENUM_STATEk(Geometry) AX_ENUM_STATEk(TessControl)
    AX_ENUM_STATEk(TessEvaluation) AX_ENUM_STATEk(Compute) AX_ENUM_REFL_END();
