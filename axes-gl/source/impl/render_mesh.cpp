#include "render_mesh.hpp"

#include <glad/glad.h>

#include <glm/gtc/type_ptr.hpp>

#include "axes/core/entt.hpp"
#include "axes/geometry/normal.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/details/gl_call.hpp"
#include "axes/gl/helpers.hpp"
#include "axes/utils/asset.hpp"
#include "axes/utils/status.hpp"
#include "glm.hpp"

namespace ax::gl {

MeshRenderer::MeshRenderer() = default;

Status MeshRenderer::Setup() {
  AX_ASSIGN_OR_RETURN(vs, Shader::CompileFile(utils::get_asset("/shader/phong/phong.vert"), ShaderType::kVertex));
  AX_ASSIGN_OR_RETURN(fs, Shader::CompileFile(utils::get_asset("/shader/phong/phong.frag"), ShaderType::kFragment));

  AX_EVAL_RETURN_NOTOK(prog_.Append(std::move(vs)).Append(std::move(fs)).Link());
  global_registry().on_destroy<Mesh>().connect<&MeshRenderer::Erase>(*this);
  AX_RETURN_OK();
}

Status MeshRenderer::TickRender() {
  AX_RETURN_NOTOK(prog_.Use());
  // TODO: Model matrix.
  auto& ctx = get_resource<Context>();
  math::mat4f model = ctx.GetGlobalModelMatrix().cast<float>();
  math::mat4f eye = math::eye<4, f32>();
  math::mat4f view = ctx.GetCamera().LookAt().cast<f32>();
  math::mat4f projection = ctx.GetCamera().GetProjectionMatrix().cast<f32>();
  math::vec3f light_pos = ctx.GetLight().position_;
  math::vec3f view_pos = ctx.GetCamera().GetPosition().cast<f32>();
  AX_CHECK_OK(prog_.SetUniform("view", view));
  AX_CHECK_OK(prog_.SetUniform("projection", projection));
  AX_CHECK_OK(prog_.SetUniform("lightPos", light_pos));
  AX_CHECK_OK(prog_.SetUniform("viewPos", view_pos));

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (auto [ent, md] : view_component<MeshRenderData>().each()) {
    if (md.use_global_model_) {
      AX_CHECK_OK(prog_.SetUniform("model", model));
    } else {
      AX_CHECK_OK(prog_.SetUniform("model", eye));
    }
    AXGL_WITH_BINDR(md.vao_) {
      math::vec4f light_coef = math::zeros<4, 1, f32>();
      if (md.is_flat_) {
        light_coef.w() = 1;
      }
      if (! md.use_lighting_) {
        light_coef.y() = light_coef.z() = 0;
        light_coef.x() = 1;
      } else {
        light_coef.x() = ctx.GetLight().ambient_strength_;
        light_coef.y() = ctx.GetLight().diffuse_strength_;
        light_coef.z() = ctx.GetLight().specular_strength_;
      }
      AX_CHECK_OK(prog_.SetUniform("lightCoefficient", light_coef));

      if (md.vao_.GetInstanceBuffer()) {
        AX_RETURN_NOTOK(md.vao_.DrawElementsInstanced(PrimitiveType::kTriangles, md.indices_.size(), Type::kUnsignedInt,
                                                      0, md.instances_.size()));
      } else {
        AX_RETURN_NOTOK(md.vao_.DrawElements(PrimitiveType::kTriangles, md.indices_.size(), Type::kUnsignedInt, 0));
      }
    }
  }
  glUseProgram(0);
  AX_RETURN_OK();
}

Status MeshRenderer::TickLogic() {
  for (auto [ent, lines] : view_component<Mesh>().each()) {
    if (lines.flush_) {
      if (has_component<MeshRenderData>(ent)) {
        remove_component<MeshRenderData>(ent);
      }
      global_registry().emplace<MeshRenderData>(ent, lines);

      AX_DLOG(INFO) << "Flushing entity: " << entt::to_integral(ent);
    }
    lines.flush_ = false;
  }
  AX_RETURN_OK();
}

Status MeshRenderer::Erase(Entity entity) {
  remove_component<MeshRenderData>(entity);
  AX_RETURN_OK();
}

Status MeshRenderer::CleanUp() {
  global_registry().clear<MeshRenderData>();
  AX_RETURN_OK();
}

MeshRenderer::~MeshRenderer() {
  AX_CHECK_OK(CleanUp());
  global_registry().on_destroy<Mesh>().disconnect<&MeshRenderer::Erase>(*this);
}

MeshRenderData::MeshRenderData(const Mesh& mesh) {
  is_flat_ = mesh.is_flat_;
  use_lighting_ = mesh.use_lighting_;
  use_global_model_ = mesh.use_global_model_;
  AX_CHECK(mesh.colors_.cols() >= mesh.vertices_.cols());
  /****************************** Prepare Buffer Data ******************************/
  vertices_.reserve(mesh.vertices_.size());
  auto normals = mesh.normals_;
  if (normals.cols() < mesh.vertices_.cols()) {
   AX_LOG(WARNING) << "Mesh Normal not set. Compute automatically";
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

  indices_.reserve(mesh.indices_.size());
  for (idx i = 0; i < mesh.indices_.cols(); i++) {
    auto index = mesh.indices_.col(i);
    indices_.push_back(index.x());
    indices_.push_back(index.y());
    indices_.push_back(index.z());
  }

  if (mesh.instance_offset_.cols() > 0) {
    AX_CHECK(mesh.instance_color_.cols() >= mesh.instance_offset_.cols());
    instances_.reserve(mesh.instance_offset_.size());
    for (idx i = 0; i < mesh.instance_offset_.cols(); i++) {
      MeshInstanceData instance;
      auto position_offset = mesh.instance_offset_.col(i);
      auto color_offset = mesh.instance_color_.col(i);
      instance.position_offset_ = glm::vec3(position_offset.x(), position_offset.y(), position_offset.z());
      instance.color_offset_ = glm::vec4(color_offset.x(), color_offset.y(), color_offset.z(), color_offset.w());
      instances_.push_back(instance);
    }
  }

  /****************************** Construct VAO ******************************/
  AX_ASSIGN_OR_DIE(vao, Vao::Create());
  vao_ = std::move(vao);

  AX_ASSIGN_OR_DIE(vbo, Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw));
  AX_ASSIGN_OR_DIE(ebo, Buffer::CreateIndexBuffer(BufferUsage::kStaticDraw));
  AXGL_WITH_BINDC(vbo) { AX_CHECK_OK(vbo.Write(vertices_)); }
  AXGL_WITH_BINDC(ebo) { AX_CHECK_OK(ebo.Write(indices_)); }
  vao_.SetIndexBuffer(std::move(ebo));
  vao_.SetVertexBuffer(std::move(vbo));
  if (instances_.size() > 0) {
    AX_ASSIGN_OR_DIE(ibo, Buffer::CreateVertexBuffer(BufferUsage::kStaticDraw));
    AXGL_WITH_BINDC(ibo) { AX_CHECK_OK(ibo.Write(instances_)); }
    vao_.SetInstanceBuffer(std::move(ibo));
  }

  const int stride = sizeof(MeshRenderVertexData);
  const int position_offset = offsetof(MeshRenderVertexData, position_);
  const int color_offset = offsetof(MeshRenderVertexData, color_);
  const int normal_offset = offsetof(MeshRenderVertexData, normal_);

  AXGL_WITH_BINDC(vao_) {
    AXGL_WITH_BINDC(vao_.GetVertexBuffer()) {
      AX_CHECK_OK(vao_.EnableAttrib(0));
      AX_CHECK_OK(vao_.SetAttribPointer(0, 3, Type::kFloat, false, stride, position_offset));
      AX_CHECK_OK(vao_.EnableAttrib(1));
      AX_CHECK_OK(vao_.SetAttribPointer(1, 4, Type::kFloat, false, stride, color_offset));
      AX_CHECK_OK(vao_.EnableAttrib(2));
      AX_CHECK_OK(vao_.SetAttribPointer(2, 3, Type::kFloat, false, stride, normal_offset));
    }

    if (instances_.size() > 0) {
      AXGL_WITH_BINDC(vao_.GetInstanceBuffer()) {
        AX_CHECK_OK(vao_.EnableAttrib(3));
        AX_CHECK_OK(vao_.SetAttribPointer(3, 3, Type::kFloat, false, sizeof(MeshInstanceData), 0));
        AX_CHECK_OK(vao_.EnableAttrib(4));
        AX_CHECK_OK(vao_.SetAttribPointer(4, 4, Type::kFloat, false, sizeof(MeshInstanceData), sizeof(glm::vec3)));
        AX_CHECK_OK(vao_.SetAttribDivisor(3, 1));
        AX_CHECK_OK(vao_.SetAttribDivisor(4, 1));
      }
    }
    AX_CHECK_OK(vao_.GetIndexBuffer().Bind());
  }

  AX_DLOG(INFO) << "MeshRenderData created: #v=" << vertices_.size() << ", #e=" << indices_.size();
}

}  // namespace ax::gl
