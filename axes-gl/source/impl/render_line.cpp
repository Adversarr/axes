#include "render_line.hpp"

#include <glad/glad.h>

#include <glm/ext.hpp>

#include "axes/core/entt.hpp"
#include "axes/gl/details/gl_call.hpp"
#include "axes/gl/helpers.hpp"
#include "axes/utils/asset.hpp"
#include "axes/utils/status.hpp"

namespace ax::gl {

LineRenderer::LineRenderer() {}

Status LineRenderer::Setup() {
  AX_ASSIGN_OR_RETURN(
      vs, Shader::CompileFile(utils::get_asset("/shader/lines/lines.vert"), ShaderType::kVertex));

  AX_ASSIGN_OR_RETURN(
      fs, Shader::CompileFile(utils::get_asset("/shader/lines/lines.frag"), ShaderType::kFragment));

  AX_EVAL_RETURN_NOTOK(prog_.Append(std::move(vs)).Append(std::move(fs)).Link());

  get_registry().on_destroy<Lines>().connect<&LineRenderer::Erase>(*this);
  AX_RETURN_OK();
}

Status LineRenderer::TickRender() {
  AX_RETURN_NOTOK(prog_.Use());
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (auto [ent, line_data] : view_component<LineRenderData>().each()) {
    AXGL_WITH_BINDR(line_data.vao_) {
      AX_RETURN_NOTOK(line_data.vao_.DrawElements(PrimitiveType::kLines, line_data.indices_.size(),
                                                  Type::kUnsignedInt, 0));
    }
  }
  glUseProgram(0);
  AX_RETURN_OK();
}

Status LineRenderer::TickLogic() {
  // Replace or add LineRenderData to the entity
  for (auto [ent, lines] : view_component<Lines>().each()) {
    if (lines.flush_) {
      if (has_component<LineRenderData>(ent)) {
        remove_component<LineRenderData>(ent);
      }
      get_registry().emplace<LineRenderData>(ent, lines);

      DLOG(INFO) << "Flushing entity: " << entt::to_integral(ent);
    }
    lines.flush_ = false;
  }
  // Remove LineRenderData from the entity, if the entity has been removed
  AX_RETURN_OK();
}

Status LineRenderer::Erase(Entity entity) {
  remove_component<LineRenderData>(entity);
  AX_RETURN_OK();
}

Status LineRenderer::CleanUp() {
  get_registry().clear<LineRenderData>();
  AX_RETURN_OK();
}

LineRenderer::~LineRenderer() {
  CHECK_OK(CleanUp());
  get_registry().on_destroy<Lines>().disconnect<&LineRenderer::Erase>(*this);
}

LineRenderData::LineRenderData(const Lines& lines) {
  vertices_.reserve(lines.vertices_.size());
  for (idx i = 0; i < lines.vertices_.cols(); i++) {
    LineRenderVertexData vertex;
    auto position = lines.vertices_.col(i);
    auto color = lines.colors_.col(i);
    vertex.vertices_ = glm::vec3(position.x(), position.y(), position.z());
    vertex.colors_ = glm::vec4(color.x(), color.y(), color.z(), color.w());
    vertices_.push_back(vertex);
  }

  indices_.reserve(lines.indices_.size());
  for (idx i = 0; i < lines.indices_.cols(); i++) {
    auto index = lines.indices_.col(i);
    indices_.push_back(index.x());
    indices_.push_back(index.y());
  }

  AX_ASSIGN_OR_DIE(vao, Vao::Create());
  vao_ = std::move(vao);

  AX_ASSIGN_OR_DIE(vbo, Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw));
  AX_ASSIGN_OR_DIE(ebo, Buffer::CreateIndexBuffer(BufferUsage::kStaticDraw));
  AXGL_WITH_BINDC(vbo) { CHECK_OK(vbo.Write(vertices_)); }
  AXGL_WITH_BINDC(ebo) { CHECK_OK(ebo.Write(indices_)); }
  vao_.SetIndexBuffer(std::move(ebo));
  vao_.SetVertexBuffer(std::move(vbo));

  const int stride = sizeof(LineRenderVertexData);
  const int position_offset = offsetof(LineRenderVertexData, vertices_);
  const int color_offset = offsetof(LineRenderVertexData, colors_);

  AXGL_WITH_BINDC(vao_) {
    AXGL_WITH_BINDC(vao_.GetVertexBuffer()) {
      CHECK_OK(vao_.SetAttribPointer(0, 3, Type::kFloat, false, stride, position_offset));
      CHECK_OK(vao_.EnableAttrib(0));
      CHECK_OK(vao_.SetAttribPointer(1, 4, Type::kFloat, false, stride, color_offset));
      CHECK_OK(vao_.EnableAttrib(1));
    }
    CHECK_OK(vao_.GetIndexBuffer().Bind());
  }
}

LineRenderData::~LineRenderData() {}

}  // namespace ax::gl
