#include "agui/details/guisys.hpp"
#include "acore/ecs/ecs.hpp"
#include "agui/details/buffers.hpp"
#include "agui/details/mesh_pipeline.hpp"
#include "agui/tags.hpp"

namespace axes::gui {

void GuiSystem::TickLogic() {
  // TODO: Foreach Simplical Render Config.

  // Foreach Simplical Render Data
  ecs::ComponentManager<SimplicalRenderData> cm;
  for (const auto &[k, v] : cm) {
    if (!v->flush_) {
      continue;
    }
    // Get the component. and fill in all the required buffers.
    CreateSceneSharedData(k, v);
  }
  SBufferFlush();
}

void GuiSystem::CreateSceneSharedData(ecs::EntityID ent,
                                      SimplicalRenderData *data) {
  ecs::ComponentManager<details::SceneRenderDataSharedGpu> sr;

  // 1. For shared part, i.e. vertex buffer, fill in directly.
  auto *shared = sr.EmplaceComponent(ent);
  size_t vertex_count = data->vertices_.Size();
  if (vertex_count == 0) {
    throw std::runtime_error("Error: No Vertex Provided for entity " +
                             std::to_string(ent));
  }
  PrepareBuffer(shared->vertex_buffer_,
                vertex_count * sizeof(details::SceneVertex));
  // CopyBuffer(shared->vertex_buffer_, )
}

void GuiSystem::SBufferAppend(void *data, size_t nbytes) {
  // Do copy, and return ok.
  char *mapped_data =
      static_cast<char *>(staging_buffer_.alloc_info_.pMappedData);
  memcpy(mapped_data + staging_buffer_usage_, data, nbytes);
  staging_buffer_usage_ += nbytes;
}

void GuiSystem::CopyBuffer(VmaAllocBuffer dst_buffer, void *data,
                           size_t nbytes) {
  size_t sbsize = staging_buffer_.alloc_info_.size;
  if (nbytes + staging_buffer_usage_ > staging_buffer_.alloc_info_.size) {
    SBufferFlush();
  }

  if (nbytes > staging_buffer_usage_) {
    // XXX: Test Required
    size_t copy_times = (nbytes + sbsize - 1) / sbsize;
    char *large_data = static_cast<char *>(data);
    for (size_t i = 0; i < copy_times; ++i) {
      // For the last turn, avoid memory error.
      size_t nb_copy = std::min(sbsize, nbytes - i * copy_times);
      SBufferAppend(large_data + i * sbsize, nb_copy);
      SBufferTransferInfo ti;
      ti.size_ = nb_copy;
      ti.dst_ = dst_buffer.buffer_;
      ti.sb_offset_ = 0;
      ti.dst_offset_ = i * sbsize;
      sb_tfinfo_.push_back(ti);
      SBufferFlush();
    }
  } else {
    SBufferTransferInfo ti;
    ti.size_ = nbytes;
    ti.dst_ = dst_buffer.buffer_;
    ti.sb_offset_ = staging_buffer_usage_;
    ti.dst_offset_ = 0;
    sb_tfinfo_.push_back(ti);
    SBufferAppend(data, nbytes);
  }
}

void GuiSystem::SBufferFlush() {
  vkc_->RunTransientCommandInplace(
      [this](vk::CommandBuffer cbuffer) {
        for (auto st : sb_tfinfo_) {
          vk::BufferCopy bc;
          bc.setSize(st.size_)
              .setDstOffset(st.dst_offset_)
              .setSrcOffset(st.sb_offset_);
          cbuffer.copyBuffer(staging_buffer_.buffer_, st.dst_, bc);
        }
      },
      vkc_->GetGraphicsQueue());
  staging_buffer_usage_ = 0;
  sb_tfinfo_.clear();
}
} // namespace axes::gui