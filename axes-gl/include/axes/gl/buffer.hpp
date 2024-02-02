#pragma once
#include <glad/glad.h>

#include "axes/core/status.hpp"
#include "axes/utils/common.hpp"
#include "details/gl_types.hpp"

namespace ax::gl {

class Buffer {
  /****************************** Ctor Dtor ******************************/
private:
  Buffer(GLuint id, BufferBindingType type, BufferUsage usage);
public:
  Buffer();
  static StatusOr<Buffer> Create(BufferBindingType type, BufferUsage usage);
  AX_DECLARE_COPY_CTOR(Buffer, delete);
  ~Buffer();
  Buffer(Buffer&& other) noexcept;
  Buffer& operator=(Buffer&& other) noexcept;

  /****************************** Methods ******************************/
  operator bool() const;

  Status Bind();
  Status Unbind();
  Status Write(const void* data, size_t size);
  Status WriteSub(const void* data, size_t size, size_t offset);

private:
  GLuint id_;
  BufferBindingType type_;
  BufferUsage usage_;
};

}  // namespace ax::gl
