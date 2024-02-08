#include "axes/gl/buffer.hpp"

#include "axes/gl/details/gl_call.hpp"

namespace ax::gl {

Buffer::Buffer(GLuint id, BufferBindingType type, BufferUsage usage)
    : id_{id}, type_{type}, usage_{usage} {}

Buffer::Buffer() : id_{0} {}

StatusOr<Buffer> Buffer::Create(BufferBindingType type, BufferUsage usage) {
  GLuint id;
  AXGL_CALLR(glGenBuffers(1, &id));
  return Buffer{id, type, usage};
}

StatusOr<Buffer> Buffer::CreateIndexBuffer(BufferUsage usage) {
  return Create(BufferBindingType::kElementArray, usage);
}

StatusOr<Buffer> Buffer::CreateVertexBuffer(BufferUsage usage) {
  return Create(BufferBindingType::kArray, usage);
}

Buffer::~Buffer() {
  if (id_) {
    glDeleteBuffers(1, &id_);
    AX_DLOG(INFO) << "Buffer " << id_ << " deleted";
  }
}

Buffer::Buffer(Buffer&& other) noexcept : id_{other.id_}, type_(other.type_), usage_(other.usage_) {
  other.id_ = 0;
}

Buffer& Buffer::operator=(Buffer&& other) noexcept {
  if (this != &other) {
    this->~Buffer();
    id_ = other.id_;
    type_ = other.type_;
    usage_ = other.usage_;
    other.id_ = 0;
  }
  return *this;
}

Buffer::operator bool() const { return id_ != 0; }

Status Buffer::Bind() {
  AXGL_CALLR(glBindBuffer(static_cast<GLenum>(type_), id_));
  AX_RETURN_OK();
}

Status Buffer::Unbind() {
  AXGL_CALLR(glBindBuffer(static_cast<GLenum>(type_), 0));
  AX_RETURN_OK();
}

Status Buffer::Write(const void* data, size_t size) {
  AXGL_CALLR(glBufferData(static_cast<GLenum>(type_), size, data, static_cast<GLenum>(usage_)));
  AX_RETURN_OK();
}

Status Buffer::WriteSub(const void* data, size_t size, size_t offset) {
  AXGL_CALLR(glBufferSubData(static_cast<GLenum>(type_), offset, size, data));
  AX_RETURN_OK();
}

}  // namespace ax::gl
