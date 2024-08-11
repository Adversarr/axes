#include "render_mesh.hpp"

#include <glad/glad.h>
#include <imgui.h>

#include <glm/gtc/type_ptr.hpp>

#include "ax/components/name.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"
#include "ax/geometry/normal.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/details/gl_call.hpp"
#include "ax/gl/helpers.hpp"
#include "ax/utils/asset.hpp"
#include "glm.hpp"

namespace ax::gl {

MeshRenderer::MeshRenderer() = default;

static void update_render(entt::registry &r, entt::entity ent) {
  if (const auto render = r.view<MeshRenderData>(); render.contains(ent)) {
    r.erase<MeshRenderData>(ent);
  }
  r.emplace<MeshRenderData>(ent, r.get<Mesh>(ent));
}

void MeshRenderer::Setup() {
  auto vs = Shader::CompileFile(utils::get_asset("/shader/phong/phong.vert"), ShaderType::kVertex);
  auto fs
      = Shader::CompileFile(utils::get_asset("/shader/phong/phong.frag"), ShaderType::kFragment);
  prog_.Append(std::move(vs)).Append(std::move(fs)).Link();
  global_registry().on_destroy<Mesh>().connect<&MeshRenderer::Erase>(*this);
  global_registry().on_update<Mesh>().connect<&update_render>();
}

void MeshRenderer::TickRender() {
  prog_.Use();
  auto& ctx = get_resource<Context>();
  math::mat4f model = ctx.GetGlobalModelMatrix().cast<float>();
  math::mat4f eye = math::eye<4, f32>();
  math::mat4f view = ctx.GetCamera().LookAt().cast<f32>();
  math::mat4f projection = ctx.GetCamera().GetProjectionMatrix().cast<f32>();
  math::vec3f light_pos = ctx.GetLight().position_;
  math::vec3f view_pos = ctx.GetCamera().GetPosition().cast<f32>();
  prog_.SetUniform("view", view);
  prog_.SetUniform("projection", projection);
  prog_.SetUniform("lightPos", light_pos);
  prog_.SetUniform("viewPos", view_pos);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (auto [ent, md] : view_component<MeshRenderData>().each()) {
    if (!md.enable_) continue;
    if (md.use_global_model_) {
      prog_.SetUniform("model", model);
    } else {
      prog_.SetUniform("model", eye);
    }
    AXGL_WITH_BIND(md.vao_) {
      math::vec4f light_coef = math::zeros<4, 1, f32>();
      if (md.is_flat_) {
        light_coef.w() = 1;
      }
      if (!md.use_lighting_) {
        light_coef.y() = light_coef.z() = 0;
        light_coef.x() = 1;
      } else {
        light_coef.x() = ctx.GetLight().ambient_strength_;
        light_coef.y() = ctx.GetLight().diffuse_strength_;
        light_coef.z() = ctx.GetLight().specular_strength_;
      }
      prog_.SetUniform("lightCoefficient", light_coef);

      if (md.vao_.GetInstanceBuffer()) {
        md.vao_.DrawElementsInstanced(PrimitiveType::kTriangles, md.indices_.size(),
                                      Type::kUnsignedInt, 0, md.instances_.size());
      } else {
        md.vao_.DrawElements(PrimitiveType::kTriangles, md.indices_.size(), Type::kUnsignedInt, 0);
      }
    }
  }
  glUseProgram(0);
}

void MeshRenderer::TickLogic() {
  // for (auto [ent, lines] : view_component<Mesh>().each()) {
  //   if (lines.flush_) {
  //     if (has_component<MeshRenderData>(ent)) {
  //       remove_component<MeshRenderData>(ent);
  //     }
  //     global_registry().emplace<MeshRenderData>(ent, lines);
  //   }
  //   lines.flush_ = false;
  // }
}

void MeshRenderer::Erase(Entity entity) { remove_component<MeshRenderData>(entity); }

void MeshRenderer::CleanUp() { global_registry().clear<MeshRenderData>(); }

MeshRenderer::~MeshRenderer() {
  CleanUp();
  global_registry().on_destroy<Mesh>().disconnect<&MeshRenderer::Erase>(*this);
}

MeshRenderData::MeshRenderData(const Mesh& mesh) {
  is_flat_ = mesh.is_flat_;
  use_lighting_ = mesh.use_lighting_;
  use_global_model_ = mesh.use_global_model_;
  AX_THROW_IF_GT(mesh.colors_.cols(), mesh.vertices_.cols(),
                 "Color size should be less than vertex size");
  /****************************** Prepare Buffer Data ******************************/
  vertices_.reserve(static_cast<size_t>(mesh.vertices_.size()));
  auto normals = mesh.normals_;
  if (normals.cols() < mesh.vertices_.cols()) {
    AX_WARN("Mesh Normal not set. Compute automatically");
    normals = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  }
  for (idx i = 0; i < mesh.vertices_.cols(); i++) {
    MeshRenderVertexData vertex;
    auto position = mesh.vertices_.col(i);
    auto color = mesh.colors_.col(i);
    auto normal = normals.col(i);
    vertex.position_ = glm::vec3(position.x(), position.y(), position.z());
    vertex.color_ = glm::vec4(color.x(), color.y(), color.z(), color.w());
    vertex.normal_ = glm::vec3(normal.x(), normal.y(), normal.z());

    vertices_.push_back(vertex);
  }

  indices_.reserve(static_cast<size_t>(mesh.indices_.size()));
  for (idx i = 0; i < mesh.indices_.cols(); i++) {
    auto index = mesh.indices_.col(i);
    indices_.push_back(math::cast<ui32>(index.x()));
    indices_.push_back(math::cast<ui32>(index.y()));
    indices_.push_back(math::cast<ui32>(index.z()));
  }

  if (mesh.instance_offset_.cols() > 0) {
    instances_.reserve(static_cast<size_t>(mesh.instance_offset_.cols()));
    for (idx i = 0; i < mesh.instance_offset_.cols(); i++) {
      MeshInstanceData instance;
      auto position_offset = mesh.instance_offset_.col(i);
      instance.position_offset_
          = glm::vec3(position_offset.x(), position_offset.y(), position_offset.z());
      if (i < mesh.instance_color_.cols()) {
        auto color_offset = mesh.instance_color_.col(i);
        instance.color_offset_
            = glm::vec4(color_offset.x(), color_offset.y(), color_offset.z(), color_offset.w());
      } else {
        instance.color_offset_ = glm::vec4(0, 0, 0, 0);
      }

      if (i < mesh.instance_scale_.cols()) {
        instance.scale_ = glm::make_vec3(mesh.instance_scale_.col(i).data());
      } else {
        instance.scale_ = glm::one<glm::vec3>();
      }

      instances_.push_back(instance);
    }
  } else {
    MeshInstanceData single;
    single.color_offset_ = glm::zero<glm::vec4>();
    single.position_offset_ = glm::zero<glm::vec4>();
    single.scale_ = glm::one<glm::vec4>();
    instances_.push_back(single);
  }

  /****************************** Construct VAO ******************************/
  vao_ = Vao::Create();

  auto vbo = Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw);
  auto ebo = Buffer::CreateIndexBuffer(BufferUsage::kStaticDraw);
  AXGL_WITH_BIND(vbo) { vbo.Write(vertices_); }
  AXGL_WITH_BIND(ebo) { ebo.Write(indices_); }
  vao_.SetIndexBuffer(std::move(ebo));
  vao_.SetVertexBuffer(std::move(vbo));
  if (instances_.size() > 0) {
    auto ibo = Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw);
    AXGL_WITH_BIND(ibo) { ibo.Write(instances_); }
    vao_.SetInstanceBuffer(std::move(ibo));
  }

  const int stride = sizeof(MeshRenderVertexData);
  const size_t position_offset = offsetof(MeshRenderVertexData, position_);
  const size_t color_offset = offsetof(MeshRenderVertexData, color_);
  const size_t normal_offset = offsetof(MeshRenderVertexData, normal_);

  AXGL_WITH_BIND(vao_) {
    AXGL_WITH_BIND(vao_.GetVertexBuffer()) {
      vao_.EnableAttrib(0);
      vao_.SetAttribPointer(0, 3, Type::kFloat, false, stride, position_offset);
      vao_.EnableAttrib(1);
      vao_.SetAttribPointer(1, 4, Type::kFloat, false, stride, color_offset);
      vao_.EnableAttrib(2);
      vao_.SetAttribPointer(2, 3, Type::kFloat, false, stride, normal_offset);
    }

    if (instances_.size() > 0) {
      AXGL_WITH_BIND(vao_.GetInstanceBuffer()) {
        vao_.EnableAttrib(3);
        vao_.SetAttribPointer(3, 3, Type::kFloat, false, sizeof(MeshInstanceData), 0);
        vao_.EnableAttrib(4);
        vao_.SetAttribPointer(4, 4, Type::kFloat, false, sizeof(MeshInstanceData),
                              sizeof(glm::vec3));

        vao_.EnableAttrib(5);
        vao_.SetAttribPointer(5, 3, Type::kFloat, false, sizeof(MeshInstanceData),
                              offsetof(MeshInstanceData, scale_));
        vao_.SetAttribDivisor(3, 1);
        vao_.SetAttribDivisor(4, 1);
        vao_.SetAttribDivisor(5, 1);
      }
    }
    vao_.GetIndexBuffer().Bind();
  }

  AX_TRACE("MeshRenderData created: #v={}, #e={}", vertices_.size(), indices_.size());
}

void MeshRenderer::RenderGui() {
  if (ImGui::TreeNode("MeshRenderer")) {
    for (auto [ent, mesh] : view_component<MeshRenderData>().each()) {
      ImGui::PushID(&mesh.enable_);
      ImGui::Checkbox("Enable", &mesh.enable_);
      ImGui::PopID();
      ImGui::SameLine();
      ImGui::Text("Entity: %d, #v=%ld, #e=%ld, #i=%ld", entt::to_integral(ent),
                  mesh.vertices_.size(), mesh.indices_.size(), mesh.instances_.size());
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
