#pragma once
#include "buffer.hpp"
#include "details/gl_types.hpp"

namespace ax::gl {

class Vao {
  /****************************** Ctor Dtor ******************************/
private:
  Vao(GLuint id);

public:
  Vao();
  ~Vao();
  Vao(Vao&& other) noexcept;
  Vao& operator=(Vao&& other) noexcept;
  AX_DECLARE_COPY_CTOR(Vao, delete);

  static StatusOr<Vao> Create();

  /****************************** Methods ******************************/
  operator bool() const;

  Status Bind();
  Status Unbind();

  /****************************** Attribute Setters ******************************/
  Status EnableAttrib(int index);
  Status SetAttribPointer(int index, int size, Type type, bool normalized, int stride, int offset);
  Status SetAttribDivisor(int index, int divisor);

  /****************************** Buffer ******************************/
  Buffer& SetIndexBuffer(Buffer&& buffer);
  Buffer& SetVertexBuffer(Buffer&& buffer);
  Buffer& SetInstanceBuffer(Buffer&& buffer);

  Buffer& GetIndexBuffer();
  Buffer& GetVertexBuffer();
  Buffer& GetInstanceBuffer();

  /****************************** Draw Calls ******************************/

  Status DrawArrays(PrimitiveType type, int first, int count);
  Status DrawElements(PrimitiveType type, int count, Type index_type, int offset);
  Status DrawElementsInstanced(PrimitiveType type, int count, Type index_type, int offset,
                               int instance_count);

private:
  Buffer index_buffer_;
  Buffer vertex_buffer_;
  Buffer instance_buffer_;
  GLuint id_;
};
}  // namespace ax::gl
