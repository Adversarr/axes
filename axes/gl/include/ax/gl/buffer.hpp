#pragma once
#include <glad/glad.h>

#include "ax/core/status.hpp"
#include "ax/utils/common.hpp"
#include "details/gl_types.hpp"

namespace ax::gl {

class Buffer {
  /****************************** Ctor Dtor ******************************/
private:
  Buffer(GLuint id, BufferBindingType type, BufferUsage usage);

public:
  Buffer();

  AX_DECLARE_COPY_CTOR(Buffer, delete);
  ~Buffer();
  Buffer(Buffer&& other) noexcept;
  Buffer& operator=(Buffer&& other) noexcept;

  /****************************** Creation ******************************/
  static StatusOr<Buffer> Create(BufferBindingType type, BufferUsage usage);
  static StatusOr<Buffer> CreateIndexBuffer(BufferUsage usage);
  static StatusOr<Buffer> CreateVertexBuffer(BufferUsage usage);

  /****************************** Method ******************************/
  operator bool() const;

  Status Bind();
  Status Unbind();
  Status Write(const void* data, size_t size);

  template <typename V> Status Write(V&& data);

  Status WriteSub(const void* data, size_t size, size_t offset);

private:
  GLuint id_;
  BufferBindingType type_;
  BufferUsage usage_;
};

template <typename V> Status Buffer::Write(V&& data) {
  size_t elem_size = sizeof(std::remove_reference_t<decltype(*std::data(std::forward<V>(data)))>);
  return Write(std::data(std::forward<V>(data)), std::size(std::forward<V>(data)) * elem_size);
}

}  // namespace ax::gl
