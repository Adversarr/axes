#include "render_line.hpp"

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

LineRenderer::LineRenderer() = default;

static void update_render(entt::registry &r, entt::entity ent) {
  if (const auto render = r.view<LineRenderData>(); render.contains(ent)) {
    r.erase<LineRenderData>(ent);
  }
  r.emplace<LineRenderData>(ent, r.get<Lines>(ent));
}

void LineRenderer::Setup() {
  auto vs = Shader::CompileFile(utils::get_asset("/shader/lines/lines.vert"), ShaderType::kVertex);
  auto fs
      = Shader::CompileFile(utils::get_asset("/shader/lines/lines.frag"), ShaderType::kFragment);
  prog_.Append(std::move(vs)).Append(std::move(fs)).Link();
  global_registry().on_destroy<Lines>().connect<&LineRenderer::Erase>(*this);
  global_registry().on_update<Lines>().connect<&update_render>();
}

void LineRenderer::TickRender() {
  prog_.Use();
  auto& ctx = get_resource<Context>();
  math::mat4f model = ctx.GetGlobalModelMatrix().cast<float>();
  math::mat4f view = ctx.GetCamera().LookAt().cast<f32>();
  math::mat4f projection = ctx.GetCamera().GetProjectionMatrix().cast<f32>();
  math::mat4f eye = math::eye<4, f32>();
  prog_.SetUniform("view", view);
  prog_.SetUniform("projection", projection);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (auto [ent, line_data] : view_component<LineRenderData>().each()) {
    if (!line_data.enable_) continue;
    if (line_data.use_global_model_) {
      prog_.SetUniform("model", model);
    } else {
      prog_.SetUniform("model", eye);
    }
    if (!line_data.instance_data_.empty()) {
      AXGL_WITH_BIND(line_data.vao_) {
        line_data.vao_.DrawElementsInstanced(PrimitiveType::kLines, line_data.indices_.size(),
                                             Type::kUnsignedInt, 0,
                                             line_data.instance_data_.size());
      }
    } else {
      AXGL_WITH_BIND(line_data.vao_) {
        line_data.vao_.DrawElements(PrimitiveType::kLines, line_data.indices_.size(),
                                    Type::kUnsignedInt, 0);
      }
    }
  }
  glUseProgram(0);
}

void LineRenderer::TickLogic() {
  // Replace or add LineRenderData to the entity
  // for (auto [ent, lines] : view_component<Lines>().each()) {
  //   if (lines.flush_) {
  //     if (has_component<LineRenderData>(ent)) {
  //       remove_component<LineRenderData>(ent);
  //     }
  //     global_registry().emplace<LineRenderData>(ent, lines);
  //   }
  //   lines.flush_ = false;
  // }
}

void LineRenderer::Erase(Entity entity) { remove_component<LineRenderData>(entity); }

void LineRenderer::CleanUp() { global_registry().clear<LineRenderData>(); }

LineRenderer::~LineRenderer() {
  CleanUp();
  global_registry().on_destroy<Lines>().disconnect<&LineRenderer::Erase>(*this);
}

LineRenderData::LineRenderData(const Lines& lines) {
  use_global_model_ = lines.use_global_model_;
  /************************* SECT: Setup Buffers *************************/
  vertices_.reserve(static_cast<size_t>(lines.vertices_.cols()));
  for (idx i = 0; i < lines.vertices_.cols(); i++) {
    LineRenderVertexData vertex;
    math::vec3r position = lines.vertices_.col(i);
    math::vec4r color = lines.colors_.col(i);
    vertex.position_ = details::to_glm(position);
    vertex.color_ = details::to_glm(color);
    vertices_.push_back(vertex);
  }

  indices_.reserve(static_cast<size_t>(lines.indices_.cols()));
  for (idx i = 0; i < lines.indices_.cols(); i++) {
    auto index = lines.indices_.col(i);
    indices_.push_back(math::cast<ui32>(index.x()));
    indices_.push_back(math::cast<ui32>(index.y()));
  }
  bool has_instance = lines.instance_offset_.size() > 0;
  if (has_instance) {
    instance_data_.reserve(static_cast<size_t>(lines.instance_offset_.cols()));
    for (idx i = 0; i < lines.instance_offset_.cols(); i++) {
      LineInstanceData instance;
      auto offset = lines.instance_offset_.col(i);
      auto color = lines.instance_color_.col(i);
      instance.offset_ = glm::vec3(offset.x(), offset.y(), offset.z());
      instance.color_ = glm::vec4(color.x(), color.y(), color.z(), color.w());

      instance_data_.push_back(instance);
    }
  }

  vao_ = Vao::Create();

  auto vbo = Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw);
  auto ebo = Buffer::CreateIndexBuffer(BufferUsage::kStaticDraw);
  AXGL_WITH_BIND(vbo) { vbo.Write(vertices_); }
  AXGL_WITH_BIND(ebo) { ebo.Write(indices_); }
  vao_.SetIndexBuffer(std::move(ebo));
  vao_.SetVertexBuffer(std::move(vbo));

  if (has_instance) {
    auto instance_vbo = Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw);
    AXGL_WITH_BIND(instance_vbo) { instance_vbo.Write(instance_data_); }
    vao_.SetInstanceBuffer(std::move(instance_vbo));
  }

  const int stride = sizeof(LineRenderVertexData);
  const size_t position_offset = offsetof(LineRenderVertexData, position_);
  const size_t color_offset = offsetof(LineRenderVertexData, color_);

  AXGL_WITH_BIND(vao_) {
    AXGL_WITH_BIND(vao_.GetVertexBuffer()) {
      vao_.EnableAttrib(0);
      vao_.SetAttribPointer(0, 3, Type::kFloat, false, stride, position_offset);
      vao_.EnableAttrib(1);
      vao_.SetAttribPointer(1, 4, Type::kFloat, false, stride, color_offset);
    }

    if (has_instance) {
      AXGL_WITH_BIND(vao_.GetInstanceBuffer()) {
        vao_.EnableAttrib(2);
        vao_.SetAttribPointer(2, 3, Type::kFloat, false, sizeof(LineInstanceData), 0);
        vao_.EnableAttrib(3);
        vao_.SetAttribPointer(3, 4, Type::kFloat, false, sizeof(LineInstanceData),
                              sizeof(glm::vec3));
        vao_.SetAttribDivisor(2, 1);
        vao_.SetAttribDivisor(3, 1);
      }
    }
    vao_.GetIndexBuffer().Bind();
  }

  AX_TRACE("LineRenderData created: #v={}, #e={}", vertices_.size(), indices_.size());
}

LineRenderData::~LineRenderData() = default;

void LineRenderer::RenderGui() {
  if (ImGui::TreeNode("LineRenderer")) {
    for (auto [ent, mesh] : view_component<LineRenderData>().each()) {
      ImGui::PushID(&mesh.enable_);
      ImGui::Checkbox("Enable", &mesh.enable_);
      ImGui::PopID();
      ImGui::SameLine();
      ImGui::Text("Entity: %d, #v=%ld, #e=%ld", entt::to_integral(ent), mesh.vertices_.size(),
                  mesh.indices_.size());
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
