#include "ax/gl/vao.hpp"

#include "ax/core/excepts.hpp"
#include "ax/gl/details/gl_call.hpp"

namespace ax::gl {

Vao::Vao() : id_{0} {}

Vao::Vao(GLuint id) : id_{id} {}

Vao::Vao(Vao&& other) noexcept
    : index_buffer_{std::move(other.index_buffer_)},
      vertex_buffer_{std::move(other.vertex_buffer_)},
      instance_buffer_{std::move(other.instance_buffer_)},
      id_{other.id_} {
  other.id_ = 0;
}

Vao& Vao::operator=(Vao&& other) noexcept {
  if (id_) {
    glDeleteVertexArrays(1, &id_);
  }
  id_ = other.id_;
  vertex_buffer_ = std::move(other.vertex_buffer_);
  index_buffer_ = std::move(other.index_buffer_);
  instance_buffer_ = std::move(other.instance_buffer_);
  other.id_ = 0;
  return *this;
}

Vao Vao::Create() {
  GLuint id;
  AXGL_CALL(glGenVertexArrays(1, &id));
  return Vao{id};
}

Vao::~Vao() {
  if (id_) {
    glDeleteVertexArrays(1, &id_);
  }
}

void Vao::Bind() { AXGL_CALL(glBindVertexArray(id_)); }

void Vao::Unbind() { AXGL_CALL(glBindVertexArray(0)); }

void Vao::SetAttribPointer(int index, int size, Type type, bool normalized, int stride,
                           size_t offset) {
  AXGL_CALL(glVertexAttribPointer(index, size, static_cast<GLenum>(type), normalized, stride,
                                  reinterpret_cast<void*>(offset)));
}

void Vao::SetAttribDivisor(int index, int divisor) {
  AXGL_CALL(glVertexAttribDivisor(index, divisor));
}

void Vao::EnableAttrib(int index) { AXGL_CALL(glEnableVertexAttribArray(index)); }

Buffer& Vao::SetIndexBuffer(Buffer&& buffer) { return index_buffer_ = std::move(buffer); }

Buffer& Vao::SetVertexBuffer(Buffer&& buffer) { return vertex_buffer_ = std::move(buffer); }

Buffer& Vao::SetInstanceBuffer(Buffer&& buffer) { return instance_buffer_ = std::move(buffer); }

Buffer& Vao::GetVertexBuffer() { return vertex_buffer_; }

Buffer& Vao::GetIndexBuffer() { return index_buffer_; }

Buffer& Vao::GetInstanceBuffer() { return instance_buffer_; }

void Vao::DrawArrays(PrimitiveType type, size_t first, size_t count) {
  AXGL_CALL(glDrawArrays(static_cast<GLenum>(type), first, count));
}

void Vao::DrawElements(PrimitiveType type, size_t count, Type index_type, size_t offset) {
  if (!index_buffer_) {
    throw make_logic_error("Index buffer is not set");
  }

  GLenum typ = static_cast<GLenum>(type), index_typ = static_cast<GLenum>(index_type);
  void* off = reinterpret_cast<void*>(offset);
  AXGL_CALL(glDrawElements(typ, count, index_typ, off));
}

void Vao::DrawElementsInstanced(PrimitiveType type, size_t count, Type index_type, size_t offset,
                                size_t instance_count) {
  if (!index_buffer_) {
    throw make_logic_error("Index buffer is not set");
  }
  if (!instance_buffer_) {
    throw make_logic_error("Instance buffer is not set");
  }
  AXGL_CALL(glDrawElementsInstanced(static_cast<GLenum>(type), count,
                                    static_cast<GLenum>(index_type),
                                    reinterpret_cast<void*>(offset), instance_count));
}

}  // namespace ax::gl
