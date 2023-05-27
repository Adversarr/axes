#include "axes/gui/details/mesh_pipeline.hpp"

#include <glm/ext/matrix_transform.hpp>

#include "axes/core/ecs/ecs.hpp"
#include "axes/core/utils/common.hpp"
#include "axes/core/utils/io.hpp"
#include "axes/core/utils/log.hpp"
#include "axes/gui/details/buffers.hpp"
#include "axes/gui/details/scene_pass.hpp"

namespace axes::gui::details {

MeshPipeline::MeshPipeline(std::shared_ptr<VkContext> vkc,
                           std::weak_ptr<VkGraphicsContext> vkg,
                           std::weak_ptr<SceneRenderPass> render_pass)
    : ScenePipelineBase(vkc, vkg, render_pass) {
  auto rp = utils::lock_or_throw(render_pass_);
  CreateUniformBuffer();
  CreateDescriptors(rp->GetDescriptorPool());
  CreatePipeline(rp->GetRenderPass());
}

MeshPipeline::~MeshPipeline(){
    // XXX: Maybe need to clean up all buffers creeated.
};

void MeshPipeline::CreatePipeline(vk::RenderPass pass) {
  vk::ShaderModule vert_module, frag_module;
  {
    auto code = utils::io::read_binary(SPV_HOME "mesh_pc.vert.spv");
    vk::ShaderModuleCreateInfo info;
    info.setPCode(reinterpret_cast<uint32_t *>(code.data()))
        .setCodeSize(code.size());
    vert_module = vkc_->GetDevice().createShaderModule(info);
    code = utils::io::read_binary(SPV_HOME "mesh_pc.frag.spv");
    info.setPCode(reinterpret_cast<uint32_t *>(code.data()))
        .setCodeSize(code.size());
    frag_module = vkc_->GetDevice().createShaderModule(info);
  }

  vk::PipelineShaderStageCreateInfo vert_stage_info;
  vert_stage_info.setStage(vk::ShaderStageFlagBits::eVertex)
      .setModule(vert_module)
      .setPName("main");
  vk::PipelineShaderStageCreateInfo frag_stage_info;
  frag_stage_info.setStage(vk::ShaderStageFlagBits::eFragment)
      .setModule(frag_module)
      .setPName("main");
  auto shader_stages = std::array{vert_stage_info, frag_stage_info};
  // Setup Vertex input
  vk::PipelineVertexInputStateCreateInfo vertex_input_create_info;
  auto vertex_binding_desc = SceneVertex::GetBindingDescriptions();
  auto vertex_attr_desc = SceneVertex::GetAttributeDescriptions();
  auto instance_binding_desc = MeshInstance::GetBindingDescriptions();
  auto instance_attr_desc = MeshInstance::GetAttributeDescriptions();
  std::move(instance_binding_desc.begin(), instance_binding_desc.end(),
            std::back_inserter(vertex_binding_desc));
  std::move(instance_attr_desc.begin(), instance_attr_desc.end(),
            std::back_inserter(vertex_attr_desc));
  vertex_input_create_info.setVertexBindingDescriptions(vertex_binding_desc)
      .setVertexAttributeDescriptions(vertex_attr_desc);

  vk::PipelineInputAssemblyStateCreateInfo input_assembly_info;
  input_assembly_info.setTopology(vk::PrimitiveTopology::eTriangleList)
      .setPrimitiveRestartEnable(VK_FALSE);

  // Setup Viewport
  vk::PipelineViewportStateCreateInfo viewport_info;
  viewport_info.setViewportCount(1).setScissorCount(1);

  // Setup Rasterization
  vk::PipelineRasterizationStateCreateInfo rasterizer_info;
  rasterizer_info.setDepthBiasEnable(VK_FALSE)
      .setRasterizerDiscardEnable(VK_FALSE)
      .setLineWidth(1.0f)
      .setPolygonMode(vk::PolygonMode::eFill)
      .setCullMode(vk::CullModeFlagBits::eNone)
      .setFrontFace(vk::FrontFace::eCounterClockwise)
      .setDepthBiasEnable(VK_FALSE);

  // Setup Multi sampling: No multisampling for better performance.
  vk::PipelineMultisampleStateCreateInfo multi_sample_info;
  multi_sample_info.setSampleShadingEnable(VK_FALSE).setRasterizationSamples(
      vk::SampleCountFlagBits::e1);

  // Setup alpha blending: We do not use color blend. although options are set
  vk::PipelineColorBlendAttachmentState color_blend_attachment;
  color_blend_attachment
      .setColorWriteMask(
          vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG
          | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA)
      .setSrcColorBlendFactor(vk::BlendFactor::eSrcAlpha)
      .setDstColorBlendFactor(vk::BlendFactor::eOneMinusSrcAlpha)
      .setColorBlendOp(vk::BlendOp::eAdd)
      .setSrcAlphaBlendFactor(vk::BlendFactor::eOne)
      .setDstAlphaBlendFactor(vk::BlendFactor::eZero)
      .setAlphaBlendOp(vk::BlendOp::eAdd)
      .setBlendEnable(VK_TRUE);

  vk::PipelineColorBlendStateCreateInfo color_blend_info;
  color_blend_info.setLogicOpEnable(VK_FALSE)
      .setLogicOp(vk::LogicOp::eCopy)
      .setAttachments(color_blend_attachment)
      .setBlendConstants({0.0f, 0.0f, 0.0f, 0.0f});

  auto dynamic_states
      = std::array{vk::DynamicState::eViewport, vk::DynamicState::eScissor};
  vk::PipelineDynamicStateCreateInfo dynamic_state_info;
  dynamic_state_info.setDynamicStates(dynamic_states);

  vk::PipelineLayoutCreateInfo pipeline_layout_info;
  pipeline_layout_info.setSetLayouts(descriptor_set_layout_);

  // Depth Testing.
  vk::PipelineDepthStencilStateCreateInfo depth_stencil;
  depth_stencil.setDepthTestEnable(VK_TRUE)
      .setDepthWriteEnable(VK_TRUE)
      .setDepthCompareOp(vk::CompareOp::eLess)
      .setStencilTestEnable(VK_FALSE);

  // setup push constants
  vk::PushConstantRange push_constant;
  // this push constant range starts at the beginning
  push_constant.offset = 0;
  // this push constant range takes up the size of a MeshPushConstants struct
  push_constant.size = sizeof(MeshPushConstants);
  // this push constant range is accessible only in the vertex shader
  push_constant.stageFlags
      = vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment;
  pipeline_layout_info.setPPushConstantRanges(&push_constant)
      .setPushConstantRangeCount(1);

  pipeline_layout_ = vkc_->GetDevice().createPipelineLayout(pipeline_layout_info);
  vk::GraphicsPipelineCreateInfo info;
  info.setStages(shader_stages)
      .setPVertexInputState(&vertex_input_create_info)
      .setPInputAssemblyState(&input_assembly_info)
      .setPViewportState(&viewport_info)
      .setPRasterizationState(&rasterizer_info)
      .setPMultisampleState(&multi_sample_info)
      .setPDepthStencilState(&depth_stencil)
      .setPColorBlendState(&color_blend_info)
      .setPDynamicState(&dynamic_state_info)
      .setLayout(pipeline_layout_)
      .setRenderPass(pass)
      .setSubpass(0)
      .setBasePipelineHandle(VK_NULL_HANDLE)
      .setBasePipelineIndex(-1);

  auto rv = vkc_->GetDevice().createGraphicsPipeline(VK_NULL_HANDLE, info);
  if (rv.result != vk::Result::eSuccess) {
    throw std::runtime_error("Failed to create pipeline. reason = \""
                             + vk::to_string(rv.result) + "\"");
  }
  pipeline_ = rv.value;
  vkc_->GetDevice().destroy(vert_module);
  vkc_->GetDevice().destroy(frag_module);
}

void MeshPipeline::CreateEntityRenderData(ecs::EntityID ent,
                                          StagingBuffer &sbuffer) {
  SimplicalRenderData *rd = ecs::ComponentManager<SimplicalRenderData>{}.Query(ent);
  if (rd == nullptr) {
    return;
  }

  // Foreach mesh
  ecs::ComponentManager<MeshRenderDataGpu> cm;
  MeshRenderDataGpu *mrd = cm.AttachOrGet(ent, vkc_);
  // Test if the mesh rendering is valid.
  mrd->is_valid_ = rd->faces_.Size() > 0;
  if (not mrd->is_valid_) {
    return;
  }

  // Prepare index buffer.
  auto index_bci = GetIndexBufferCreateInfo();
  mrd->index_count_ = rd->faces_.Size();
  index_bci.setSize(mrd->index_count_ * sizeof(uint32_t));
  vkc_->PrepareBuffer(mrd->index_buffer_, index_bci,
                      GetIndexBufferAllocationInfo());
  sbuffer.CopyBuffer(mrd->index_buffer_, rd->faces_.Ptr(),
                     mrd->index_count_ * sizeof(uint32_t));

  // Prepare instance buffer
  auto instance_bci = GetVertexBufferCreateInfo();
  mrd->instance_count_ = rd->instances_.Size();
  if (mrd->instance_count_ == 0) {
    throw std::range_error("Instance count = 0 is not valid currently.");
  }
  instance_bci.setSize(mrd->instance_count_ * sizeof(rd->instances_[0]));
  vkc_->PrepareBuffer(mrd->instance_buffer_, instance_bci,
                      GetVertexBufferAllocationInfo());
  sbuffer.CopyBuffer(mrd->instance_buffer_, rd->instances_.Ptr(),
                     instance_bci.size);

  mrd->is_valid_ = true;
  mrd->pc_.model_ = glm::identity<glm::mat4>();
  AXES_TRACE(
      "MeshPipeline: Create Data for Entity {}, with instance count {}, index "
      "count {}",
      ent, mrd->instance_count_, mrd->index_count_);
}

void MeshPipeline::Draw(vk::CommandBuffer &cbuffer) {
  // Bind Pipeline
  auto vkgl = utils::lock_or_throw(vkg_);
  auto extent = vkgl->GetSwapchainExtent();
  cbuffer.bindPipeline(vk::PipelineBindPoint::eGraphics, pipeline_);
  // Setup Viewport and scissor.
  vk::Viewport viewport(0.0, 0.0, extent.width, extent.height, 0.0, 1.0);
  cbuffer.setViewport(0, viewport);
  vk::Rect2D scissor{{0, 0}, extent};
  cbuffer.setScissor(0, scissor);

  // Bind UBO inline.
  cbuffer.bindDescriptorSets(vk::PipelineBindPoint::eGraphics, pipeline_layout_, 0,
                             ubo_descriptor_sets_[vkgl->GetFrameIndex()], {});
  ecs::ComponentManager<MeshRenderDataGpu> manager;
  for (auto [ent, render_data] : manager) {
    auto *shared
        = ecs::ComponentManager<SceneRenderDataSharedGpu>{}.Query(ent);
    if (!render_data->is_valid_) {
      continue;
    }
    vk::Buffer vb = shared->vertex_buffer_.buffer_;
    vk::Buffer isb = render_data->instance_buffer_.buffer_;
    cbuffer.bindVertexBuffers(0, vb, static_cast<vk::DeviceSize>(0));
    cbuffer.bindVertexBuffers(1, isb, static_cast<vk::DeviceSize>(0));
    cbuffer.bindIndexBuffer(render_data->index_buffer_.buffer_, 0,
                            vk::IndexType::eUint32);
    cbuffer.pushConstants(
        pipeline_layout_,
        vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment, 0,
        sizeof(MeshPushConstants), &render_data->pc_);
    cbuffer.drawIndexed(render_data->index_count_, render_data->instance_count_, 0,
                        0, 0);
  }
}

}  // namespace axes::gui::details
