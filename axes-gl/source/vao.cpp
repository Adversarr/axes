#include "axes/gl/vao.hpp"

#include "axes/gl/details/gl_call.hpp"
#include "axes/utils/status.hpp"

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

StatusOr<Vao> Vao::Create() {
  GLuint id;
  AXGL_CALLR(glGenVertexArrays(1, &id));
  return Vao{id};
}

Vao::~Vao() {
  if (id_) {
    glDeleteVertexArrays(1, &id_);
    AX_DLOG(INFO) << "Vao " << id_ << " deleted";
  }
}

Status Vao::Bind() {
  AXGL_CALLR(glBindVertexArray(id_));
  AX_RETURN_OK();
}

Status Vao::Unbind() {
  AXGL_CALLR(glBindVertexArray(0));
  AX_RETURN_OK();
}

Status Vao::SetAttribPointer(int index, int size, Type type, bool normalized, int stride,
                             int offset) {
  AXGL_CALLR(glVertexAttribPointer(index, size, static_cast<GLenum>(type), normalized, stride,
                                   reinterpret_cast<void*>(offset)));
  AX_RETURN_OK();
}

Status Vao::SetAttribDivisor(int index, int divisor) {
  AXGL_CALLR(glVertexAttribDivisor(index, divisor));
  AX_RETURN_OK();
}

Status Vao::EnableAttrib(int index) {
  AXGL_CALLR(glEnableVertexAttribArray(index));
  AX_RETURN_OK();
}

Buffer& Vao::SetIndexBuffer(Buffer&& buffer) { return index_buffer_ = std::move(buffer); }

Buffer& Vao::SetVertexBuffer(Buffer&& buffer) { return vertex_buffer_ = std::move(buffer); }

Buffer& Vao::SetInstanceBuffer(Buffer&& buffer) { return instance_buffer_ = std::move(buffer); }

Buffer& Vao::GetVertexBuffer() { return vertex_buffer_; }

Buffer& Vao::GetIndexBuffer() { return index_buffer_; }

Buffer& Vao::GetInstanceBuffer() { return instance_buffer_; }

Status Vao::DrawArrays(PrimitiveType type, int first, int count) {
  AXGL_CALLR(glDrawArrays(static_cast<GLenum>(type), first, count));
  AX_RETURN_OK();
}

Status Vao::DrawElements(PrimitiveType type, int count, Type index_type, int offset) {
  if (!index_buffer_) {
    return utils::InvalidArgumentError("Index buffer is not set");
  }
  AXGL_CALLR(glDrawElements(static_cast<GLenum>(type), count, static_cast<GLenum>(index_type),
                            reinterpret_cast<void*>(offset)));
  AX_RETURN_OK();
}

Status Vao::DrawElementsInstanced(PrimitiveType type, int count, Type index_type, int offset,
                                  int instance_count) {
  if (!index_buffer_) {
    return utils::InvalidArgumentError("Index buffer is not set");
  }
  if (!instance_buffer_) {
    return utils::InvalidArgumentError("Instance buffer is not set");
  }
  AXGL_CALLR(glDrawElementsInstanced(static_cast<GLenum>(type), count,
                                     static_cast<GLenum>(index_type),
                                     reinterpret_cast<void*>(offset), instance_count));
  AX_RETURN_OK();
}

}  // namespace ax::gl
