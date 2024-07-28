#include "render_point.hpp"

#include <glad/glad.h>
#include <imgui.h>

#include <glm/gtc/type_ptr.hpp>

#include "ax/components/name.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/logging.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/details/gl_call.hpp"
#include "ax/gl/helpers.hpp"
#include "ax/utils/asset.hpp"
#include "glm.hpp"

namespace ax::gl {

PointRenderer::PointRenderer() {}

void PointRenderer::Setup() {
  auto vs
      = Shader::CompileFile(utils::get_asset("/shader/points/points.vert"), ShaderType::kVertex);
  auto fs
      = Shader::CompileFile(utils::get_asset("/shader/points/points.frag"), ShaderType::kFragment);

  prog_.Append(std::move(vs)).Append(std::move(fs)).Link();
  global_registry().on_destroy<Points>().connect<&PointRenderer::Erase>(*this);
}

void PointRenderer::TickRender() {
  bool is_pg_point_size = glIsEnabled(GL_PROGRAM_POINT_SIZE);
  if (!is_pg_point_size) {
    AXGL_CALL(glEnable(GL_PROGRAM_POINT_SIZE));
  }
  prog_.Use();
  auto& ctx = get_resource<Context>();
  math::mat4f model = ctx.GetGlobalModelMatrix().cast<float>();
  math::mat4f view = ctx.GetCamera().LookAt().cast<f32>();
  math::mat4f projection = ctx.GetCamera().GetProjectionMatrix().cast<f32>();
  math::mat4f eye = math::eye<4, f32>();
  prog_.SetUniform("view", view);
  prog_.SetUniform("projection", projection);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (auto [ent, point_data] : view_component<PointRenderData>().each()) {
    if (!point_data.enable_) {
      continue;
    }
    prog_.SetUniform("psize", point_data.point_size_);
    if (point_data.use_global_model_) {
      prog_.SetUniform("model", model);
    } else {
      prog_.SetUniform("model", eye);
    }

    AXGL_WITH_BIND(point_data.vao_) {
      point_data.vao_.DrawArrays(PrimitiveType::kPoints, 0, point_data.vertices_.size());
    }
  }
  glUseProgram(0);

  if (!is_pg_point_size) {
    AXGL_CALL(glDisable(GL_PROGRAM_POINT_SIZE));
  }
}

void PointRenderer::TickLogic() {
  // Replace or add PointRenderData to the entity
  for (auto [ent, points] : view_component<Points>().each()) {
    if (points.flush_) {
      if (has_component<PointRenderData>(ent)) {
        remove_component<PointRenderData>(ent);
      }
      global_registry().emplace<PointRenderData>(ent, points);

      AX_TRACE("Flushing entity: {}", entt::to_integral(ent));
    }
    points.flush_ = false;
  }
}

void PointRenderer::Erase(Entity entity) { remove_component<PointRenderData>(entity); }

void PointRenderer::CleanUp() { global_registry().clear<PointRenderData>(); }

PointRenderer::~PointRenderer() {
  CleanUp();
  global_registry().on_destroy<Points>().disconnect<&PointRenderer::Erase>(*this);
}

PointRenderData::PointRenderData(const Points& point) {
  use_global_model_ = point.use_global_model_;
  /************************* SECT: Setup Buffers *************************/
  vertices_.reserve(static_cast<size_t>(point.vertices_.size()));
  for (idx i = 0; i < point.vertices_.cols(); i++) {
    PointRenderVertexData vertex;
    auto position = point.vertices_.col(i);
    auto color = point.colors_.col(i);
    vertex.position_ = glm::vec3(position.x(), position.y(), position.z());
    vertex.color_ = glm::vec4(color.x(), color.y(), color.z(), color.w());
    vertices_.push_back(vertex);
  }

  vao_ = Vao::Create();

  auto vbo = Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw);
  AXGL_WITH_BIND(vbo) { vbo.Write(vertices_); }
  vao_.SetVertexBuffer(std::move(vbo));

  const int stride = sizeof(PointRenderVertexData);
  const int position_offset = offsetof(PointRenderVertexData, position_);
  const int color_offset = offsetof(PointRenderVertexData, color_);

  AXGL_WITH_BIND(vao_) {
    AXGL_WITH_BIND(vao_.GetVertexBuffer()) {
      vao_.EnableAttrib(0);
      vao_.SetAttribPointer(0, 3, Type::kFloat, false, stride, position_offset);
      vao_.EnableAttrib(1);
      vao_.SetAttribPointer(1, 4, Type::kFloat, false, stride, color_offset);
    }
  }

  point_size_ = static_cast<f32>(point.point_size_);
  AX_DEBUG("PointRenderData created: #v={}", vertices_.size());
}

PointRenderData::~PointRenderData() = default;

void PointRenderer::RenderGui() {
  if (ImGui::TreeNode("PointRenderer")) {
    for (auto [ent, point_data] : view_component<PointRenderData>().each()) {
      ImGui::PushID(static_cast<int>(entt::to_integral(ent)));
      ImGui::Checkbox("Enable", &point_data.enable_);
      ImGui::PopID();

      ImGui::SameLine();
      ImGui::Text("Entity: %d, #v=%ld", static_cast<int>(entt::to_integral(ent)),
                  point_data.vertices_.size());
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
