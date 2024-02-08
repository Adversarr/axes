#include "render_line.hpp"

#include <glad/glad.h>

#include <glm/gtc/type_ptr.hpp>

#include "axes/core/entt.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/details/gl_call.hpp"
#include "axes/gl/helpers.hpp"
#include "axes/utils/asset.hpp"
#include "axes/utils/status.hpp"
#include "glm.hpp"

namespace ax::gl {

LineRenderer::LineRenderer() {}

Status LineRenderer::Setup() {
  AX_ASSIGN_OR_RETURN(vs, Shader::CompileFile(utils::get_asset("/shader/lines/lines.vert"), ShaderType::kVertex));

  AX_ASSIGN_OR_RETURN(fs, Shader::CompileFile(utils::get_asset("/shader/lines/lines.frag"), ShaderType::kFragment));

  AX_EVAL_RETURN_NOTOK(prog_.Append(std::move(vs)).Append(std::move(fs)).Link());

  global_registry().on_destroy<Lines>().connect<&LineRenderer::Erase>(*this);
  AX_RETURN_OK();
}

Status LineRenderer::TickRender() {
  AX_RETURN_NOTOK(prog_.Use());
  auto& ctx = get_resource<Context>();
  math::mat4f model = ctx.GetGlobalModelMatrix().cast<float>();
  math::mat4f view = ctx.GetCamera().LookAt().cast<f32>();
  math::mat4f projection = ctx.GetCamera().GetProjectionMatrix().cast<f32>();
  math::mat4f eye = math::eye<4, f32>();
  AX_CHECK_OK(prog_.SetUniform("view", view));
  AX_CHECK_OK(prog_.SetUniform("projection", projection));
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (auto [ent, line_data] : view_component<LineRenderData>().each()) {
    if (line_data.use_global_model_) {
      AX_CHECK_OK(prog_.SetUniform("model", model));
    } else {
      AX_CHECK_OK(prog_.SetUniform("model", eye));
    }
    if (line_data.instance_data_.size() > 0) {
      AXGL_WITH_BINDR(line_data.vao_) {
        AX_RETURN_NOTOK(line_data.vao_.DrawElementsInstanced(PrimitiveType::kLines, line_data.indices_.size(),
                                                             Type::kUnsignedInt, 0, line_data.instance_data_.size()));
      }
    } else {
      AXGL_WITH_BINDR(line_data.vao_) {
        AX_RETURN_NOTOK(
            line_data.vao_.DrawElements(PrimitiveType::kLines, line_data.indices_.size(), Type::kUnsignedInt, 0));
      }
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
      global_registry().emplace<LineRenderData>(ent, lines);

      AX_DLOG(INFO) << "Flushing entity: " << entt::to_integral(ent);
    }
    lines.flush_ = false;
  }
  AX_RETURN_OK();
}

Status LineRenderer::Erase(Entity entity) {
  remove_component<LineRenderData>(entity);
  AX_RETURN_OK();
}

Status LineRenderer::CleanUp() {
  global_registry().clear<LineRenderData>();
  AX_RETURN_OK();
}

LineRenderer::~LineRenderer() {
  AX_CHECK_OK(CleanUp());
  global_registry().on_destroy<Lines>().disconnect<&LineRenderer::Erase>(*this);
}

LineRenderData::LineRenderData(const Lines& lines) {
  use_global_model_ = lines.use_global_model_;
  /************************* SECT: Setup Buffers *************************/
  vertices_.reserve(lines.vertices_.size());
  for (idx i = 0; i < lines.vertices_.cols(); i++) {
    LineRenderVertexData vertex;
    auto position = lines.vertices_.col(i);
    auto color = lines.colors_.col(i);
    vertex.position_ = glm::vec3(position.x(), position.y(), position.z());
    vertex.color_ = glm::vec4(color.x(), color.y(), color.z(), color.w());

    vertices_.push_back(vertex);
  }

  indices_.reserve(lines.indices_.size());
  for (idx i = 0; i < lines.indices_.cols(); i++) {
    auto index = lines.indices_.col(i);
    indices_.push_back(index.x());
    indices_.push_back(index.y());
  }
  bool has_instance = lines.instance_offset_.size() > 0;
  if (has_instance) {
    instance_data_.reserve(lines.instance_offset_.cols());
    for (idx i = 0; i < lines.instance_offset_.cols(); i++) {
      LineInstanceData instance;
      auto offset = lines.instance_offset_.col(i);
      auto color = lines.instance_color_.col(i);
      instance.offset_ = glm::vec3(offset.x(), offset.y(), offset.z());
      instance.color_ = glm::vec4(color.x(), color.y(), color.z(), color.w());

      instance_data_.push_back(instance);
    }
  }

  AX_ASSIGN_OR_DIE(vao, Vao::Create());
  vao_ = std::move(vao);

  AX_ASSIGN_OR_DIE(vbo, Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw));
  AX_ASSIGN_OR_DIE(ebo, Buffer::CreateIndexBuffer(BufferUsage::kStaticDraw));
  AXGL_WITH_BINDC(vbo) { AX_CHECK_OK(vbo.Write(vertices_)); }
  AXGL_WITH_BINDC(ebo) { AX_CHECK_OK(ebo.Write(indices_)); }
  vao_.SetIndexBuffer(std::move(ebo));
  vao_.SetVertexBuffer(std::move(vbo));

  if (has_instance) {
    AX_ASSIGN_OR_DIE(instance_vbo, Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw));
    AXGL_WITH_BINDC(instance_vbo) { AX_CHECK_OK(instance_vbo.Write(instance_data_)); }
    vao_.SetInstanceBuffer(std::move(instance_vbo));
  }

  const int stride = sizeof(LineRenderVertexData);
  const int position_offset = offsetof(LineRenderVertexData, position_);
  const int color_offset = offsetof(LineRenderVertexData, color_);

  AXGL_WITH_BINDC(vao_) {
    AXGL_WITH_BINDC(vao_.GetVertexBuffer()) {
      AX_CHECK_OK(vao_.EnableAttrib(0));
      AX_CHECK_OK(vao_.SetAttribPointer(0, 3, Type::kFloat, false, stride, position_offset));
      AX_CHECK_OK(vao_.EnableAttrib(1));
      AX_CHECK_OK(vao_.SetAttribPointer(1, 4, Type::kFloat, false, stride, color_offset));
    }

    if (has_instance) {
      AXGL_WITH_BINDC(vao_.GetInstanceBuffer()) {
        AX_CHECK_OK(vao_.EnableAttrib(2));
        AX_CHECK_OK(vao_.SetAttribPointer(2, 3, Type::kFloat, false, sizeof(LineInstanceData), 0));
        AX_CHECK_OK(vao_.EnableAttrib(3));
        AX_CHECK_OK(vao_.SetAttribPointer(3, 4, Type::kFloat, false, sizeof(LineInstanceData), sizeof(glm::vec3)));
        AX_CHECK_OK(vao_.SetAttribDivisor(2, 1));
        AX_CHECK_OK(vao_.SetAttribDivisor(3, 1));
      }
    }
    AX_CHECK_OK(vao_.GetIndexBuffer().Bind());
  }

  AX_DLOG(INFO) << "LineRenderData created: #v=" << vertices_.size() << ", #e=" << indices_.size();
}

LineRenderData::~LineRenderData() {}

}  // namespace ax::gl
