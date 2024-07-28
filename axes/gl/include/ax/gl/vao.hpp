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

  static Vao Create();

  /****************************** Methods ******************************/
  operator bool() const;

  void Bind();
  void Unbind();

  /****************************** Attribute Setters ******************************/
  void EnableAttrib(int index);
  void SetAttribPointer(int index, int size, Type type, bool normalized, int stride, size_t offset);
  void SetAttribDivisor(int index, int divisor);

  /****************************** Buffer ******************************/
  Buffer& SetIndexBuffer(Buffer&& buffer);
  Buffer& SetVertexBuffer(Buffer&& buffer);
  Buffer& SetInstanceBuffer(Buffer&& buffer);

  Buffer& GetIndexBuffer();
  Buffer& GetVertexBuffer();
  Buffer& GetInstanceBuffer();

  /****************************** Draw Calls ******************************/

  void DrawArrays(PrimitiveType type, size_t first, size_t count);
  void DrawElements(PrimitiveType type, size_t count, Type index_type, size_t offset);
  void DrawElementsInstanced(PrimitiveType type, size_t count, Type index_type, size_t offset,
                               size_t instance_count);

private:
  Buffer index_buffer_;
  Buffer vertex_buffer_;
  Buffer instance_buffer_;
  GLuint id_;
};
}  // namespace ax::gl
