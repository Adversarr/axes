#include "render_quiver.hpp"

#include <imgui.h>

#include "ax/components/name.hpp"
#include "ax/core/logging.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/details/gl_call.hpp"
#include "ax/gl/helpers.hpp"
#include "ax/gl/program.hpp"
#include "ax/utils/asset.hpp"

namespace ax::gl {

static void update_render(entt::registry &r, entt::entity ent) {
  if (const auto render = r.view<QuiverRenderData>(); render.contains(ent)) {
    r.erase<QuiverRenderData>(ent);
  }
  r.emplace<QuiverRenderData>(ent, r.get<Quiver>(ent));
}

QuiverRenderer::QuiverRenderer() = default;

void QuiverRenderer::Setup() {
  auto vs = Shader::CompileFile(utils::get_asset("/shader/lines/lines.vert"), ShaderType::kVertex);
  auto fs
      = Shader::CompileFile(utils::get_asset("/shader/lines/lines.frag"), ShaderType::kFragment);

  prog_.Append(std::move(vs)).Append(std::move(fs)).Link();

  global_registry().on_destroy<Quiver>().connect<&QuiverRenderer::Erase>(*this);
  global_registry().on_update<Quiver>().connect<&update_render>();
}

QuiverRenderer::~QuiverRenderer() {
  global_registry().on_destroy<Quiver>().disconnect<&QuiverRenderer::Erase>(*this);
}

void QuiverRenderer::TickRender() {
  prog_.Use();
  auto& ctx = get_resource<Context>();
  math::mat4f model = ctx.GetGlobalModelMatrix().cast<float>();
  math::mat4f view = ctx.GetCamera().LookAt().cast<f32>();
  math::mat4f projection = ctx.GetCamera().GetProjectionMatrix().cast<f32>();
  math::mat4f eye = math::eye<4, f32>();
  prog_.SetUniform("view", view);
  prog_.SetUniform("projection", projection);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (auto [ent, quiver_data] : view_component<QuiverRenderData>().each()) {
    if (!quiver_data.enable_) {
      continue;
    }
    if (quiver_data.use_global_model_) {
      prog_.SetUniform("model", model);
    } else {
      prog_.SetUniform("model", eye);
    }
    AXGL_WITH_BIND(quiver_data.vao_) {
      quiver_data.vao_.DrawArrays(PrimitiveType::kLines, 0, quiver_data.vertices_.size());
    }
  }
  glUseProgram(0);
}

void QuiverRenderer::TickLogic() {
  // for (auto [ent, quiver] : view_component<Quiver>().each()) {
  //   if (quiver.flush_) {
  //     if (has_component<QuiverRenderData>(ent)) {
  //       remove_component<QuiverRenderData>(ent);
  //     }
  //     add_component<QuiverRenderData>(ent, quiver);
  //     AX_TRACE("Flushing entity: {}", entt::to_integral(ent));
  //   }
  //   quiver.flush_ = false;
  // }
}

void QuiverRenderer::Erase(Entity entity) {
  if (has_component<QuiverRenderData>(entity)) {
    remove_component<QuiverRenderData>(entity);
  }
}

void QuiverRenderer::CleanUp() { global_registry().clear<QuiverRenderData>(); }

QuiverRenderData::QuiverRenderData(Quiver const& quiver) {
  vertices_.resize(static_cast<size_t>(quiver.positions_.cols() * 2));
  for (auto i = 0; i < quiver.positions_.cols(); ++i) {
    auto position = quiver.positions_.col(i);
    auto color = quiver.colors_.col(i);
    math::vec3r direction = quiver.directions_.col(i);
    if (quiver.normalize_) {
      direction.normalize();
    }
    auto end = position + direction * quiver.scale_;

    size_t cur = static_cast<size_t>(i * 2), cur_next = static_cast<size_t>(i * 2 + 1);
    vertices_[cur].position_ = glm::vec3(position.x(), position.y(), position.z());
    vertices_[cur].color_ = glm::vec4(color.x(), color.y(), color.z(), color.w());

    vertices_[cur_next].position_ = glm::vec3(end.x(), end.y(), end.z());
    vertices_[cur_next].color_ = glm::vec4(color.x(), color.y(), color.z(), color.w());
    vertices_[cur_next].color_ *= quiver.head_ratio_;
  }

  vao_ = Vao::Create();

  auto vbo = Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw);
  AXGL_WITH_BIND(vbo) { vbo.Write(vertices_); }
  vao_.SetVertexBuffer(std::move(vbo));

  const int stride = sizeof(QuiverRenderVertexData);
  const size_t position_offset = offsetof(QuiverRenderVertexData, position_);
  const size_t color_offset = offsetof(QuiverRenderVertexData, color_);

  AXGL_WITH_BIND(vao_) {
    AXGL_WITH_BIND(vao_.GetVertexBuffer()) {
      vao_.EnableAttrib(0);
      vao_.SetAttribPointer(0, 3, Type::kFloat, false, stride, position_offset);
      vao_.EnableAttrib(1);
      vao_.SetAttribPointer(1, 4, Type::kFloat, false, stride, color_offset);
    }
  }

  AX_TRACE("QuiverRenderData created: #v={}", vertices_.size());
}

QuiverRenderData::~QuiverRenderData() = default;

void QuiverRenderer::RenderGui() {
  if (ImGui::TreeNode("QuiverRenderer")) {
    for (auto [ent, quiver] : view_component<QuiverRenderData>().each()) {
      ImGui::PushID(static_cast<int>(entt::to_integral(ent)));
      ImGui::Checkbox("Enable", &quiver.enable_);
      ImGui::PopID();
      ImGui::SameLine();
      ImGui::Text("Entity: %d, #v=%ld", static_cast<int>(entt::to_integral(ent)),
                  quiver.vertices_.size());

      auto* name = try_get_component<cmpt::Name>(ent);
      if (name != nullptr) {
        ImGui::SameLine();
        ImGui::Text("Name: %s", name->value_.c_str());
      }
    }
    ImGui::TreePop();
  }
}

}  // namespace ax::gl
