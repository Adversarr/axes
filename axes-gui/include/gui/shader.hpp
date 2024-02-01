#pragma once
#include <GL/gl.h>

#include <axes/utils/common.hpp>
#include <axes/utils/enum_refl.hpp>

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
  Shader(const char* shader_content, ShaderType shader_type);
  AX_DECLARE_CONSTRUCTOR(Shader, delete, delete);


  operator bool() const;

  GLuint GetId() const;

private:
  GLuint shader_id_{0};
  ShaderType shader_type_{ShaderType::kVertex};
};

}  // namespace ax::gui

AX_ENUM_REFL_BEGIN(ax::gui::ShaderType)
  AX_ENUM_STATEk(Vertex)
  AX_ENUM_STATEk(Fragment)
  AX_ENUM_STATEk(Geometry)
  AX_ENUM_STATEk(TessControl)
  AX_ENUM_STATEk(TessEvaluation)
  AX_ENUM_STATEk(Compute)
AX_ENUM_REFL_END();
