#include "render_quiver.hpp"

#include "axes/gl/context.hpp"
#include "axes/gl/details/gl_call.hpp"
#include "axes/gl/helpers.hpp"
#include "axes/gl/program.hpp"
#include "axes/utils/asset.hpp"
#include "axes/utils/status.hpp"

namespace ax::gl {

QuiverRenderer::QuiverRenderer() {}

Status QuiverRenderer::Setup() {
  AX_ASSIGN_OR_RETURN(
      vs, Shader::CompileFile(utils::get_asset("/shader/lines/lines.vert"), ShaderType::kVertex));

  AX_ASSIGN_OR_RETURN(
      fs, Shader::CompileFile(utils::get_asset("/shader/lines/lines.frag"), ShaderType::kFragment));

  AX_EVAL_RETURN_NOTOK(prog_.Append(std::move(vs)).Append(std::move(fs)).Link());

  global_registry().on_destroy<Quiver>().connect<&QuiverRenderer::Erase>(*this);
  global_registry().on_destroy<Quiver>().connect<&QuiverRenderer::Erase>(*this);
  AX_RETURN_OK();
}

QuiverRenderer::~QuiverRenderer() {
  global_registry().on_destroy<Quiver>().disconnect<&QuiverRenderer::Erase>(*this);
}

Status QuiverRenderer::TickRender() {
  AX_RETURN_NOTOK(prog_.Use());
  auto& ctx = get_resource<Context>();
  math::mat4f model = ctx.GetGlobalModelMatrix().cast<float>();
  math::mat4f view = ctx.GetCamera().LookAt().cast<f32>();
  math::mat4f projection = ctx.GetCamera().GetProjectionMatrix().cast<f32>();
  math::mat4f eye = math::eye<4, f32>();
  AX_CHECK_OK(prog_.SetUniform("view", view));
  AX_CHECK_OK(prog_.SetUniform("projection", projection));
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (auto [ent, quiver_data] : view_component<QuiverRenderData>().each()) {
    if (quiver_data.use_global_model_) {
      AX_CHECK_OK(prog_.SetUniform("model", model));
    } else {
      AX_CHECK_OK(prog_.SetUniform("model", eye));
    }
    AXGL_WITH_BINDR(quiver_data.vao_) {
      AX_RETURN_NOTOK(
          quiver_data.vao_.DrawArrays(PrimitiveType::kLines, 0, quiver_data.vertices_.size()));
    }
  }
  glUseProgram(0);
  AX_RETURN_OK();
}

Status QuiverRenderer::TickLogic() {
  for (auto [ent, quiver] : view_component<Quiver>().each()) {
    if (quiver.flush_) {
      if (has_component<QuiverRenderData>(ent)) {
        remove_component<QuiverRenderData>(ent);
      }
      add_component<QuiverRenderData>(ent, quiver);

      AX_DLOG(INFO) << "Flushing entity: " << entt::to_integral(ent);
    }
    quiver.flush_ = false;
  }

  AX_RETURN_OK();
}

Status QuiverRenderer::Erase(Entity entity) {
  if (has_component<QuiverRenderData>(entity)) {
    remove_component<QuiverRenderData>(entity);
  }
  AX_RETURN_OK();
}

Status QuiverRenderer::CleanUp() {
  global_registry().clear<QuiverRenderData>();
  AX_RETURN_OK();
}

QuiverRenderData::QuiverRenderData(Quiver const& quiver) {
  vertices_.resize(quiver.positions_.cols() * 2);
  for (auto i = 0; i < quiver.positions_.cols(); ++i) {
    auto position = quiver.positions_.col(i);
    auto color = quiver.colors_.col(i);
    math::vec3r direction = quiver.directions_.col(i);
    if (quiver.normalize_) {
      direction.normalize();
    }
    auto end = position + direction * quiver.scale_;

    vertices_[i * 2].position_ = glm::vec3(position.x(), position.y(), position.z());
    vertices_[i * 2].color_ = glm::vec4(color.x(), color.y(), color.z(), color.w());

    vertices_[i * 2 + 1].position_ = glm::vec3(end.x(), end.y(), end.z());
    vertices_[i * 2 + 1].color_ = glm::vec4(color.x(), color.y(), color.z(), color.w());
    vertices_[i * 2 + 1].color_ *= quiver.head_ratio_;
  }

  AX_ASSIGN_OR_DIE(vao, Vao::Create());
  vao_ = std::move(vao);

  AX_ASSIGN_OR_DIE(vbo, Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw));
  AXGL_WITH_BINDC(vbo) { AX_CHECK_OK(vbo.Write(vertices_)); }
  vao_.SetVertexBuffer(std::move(vbo));

  const int stride = sizeof(QuiverRenderVertexData);
  const int position_offset = offsetof(QuiverRenderVertexData, position_);
  const int color_offset = offsetof(QuiverRenderVertexData, color_);

  AXGL_WITH_BINDC(vao_) {
    AXGL_WITH_BINDC(vao_.GetVertexBuffer()) {
      AX_CHECK_OK(vao_.EnableAttrib(0));
      AX_CHECK_OK(vao_.SetAttribPointer(0, 3, Type::kFloat, false, stride, position_offset));
      AX_CHECK_OK(vao_.EnableAttrib(1));
      AX_CHECK_OK(vao_.SetAttribPointer(1, 4, Type::kFloat, false, stride, color_offset));
    }
  }

  AX_DLOG(INFO) << "QuiverRenderData created: #v=" << vertices_.size();
}

QuiverRenderData::~QuiverRenderData() {}

}  // namespace ax::gl