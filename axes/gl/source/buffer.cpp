#include "ax/gl/buffer.hpp"

#include "ax/gl/details/gl_call.hpp"

namespace ax::gl {

Buffer::Buffer(GLuint id, BufferBindingType type, BufferUsage usage)
    : id_{id}, type_{type}, usage_{usage} {}

Buffer::Buffer() : id_{0} {}

Buffer Buffer::Create(BufferBindingType type, BufferUsage usage) {
  GLuint id;
  AXGL_CALL(glGenBuffers(1, &id));
  return Buffer{id, type, usage};
}

Buffer Buffer::CreateIndexBuffer(BufferUsage usage) {
  return Create(BufferBindingType::kElementArray, usage);
}

Buffer Buffer::CreateVertexBuffer(BufferUsage usage) {
  return Create(BufferBindingType::kArray, usage);
}

Buffer::~Buffer() {
  if (id_) {
    glDeleteBuffers(1, &id_);
  }
}

Buffer::Buffer(Buffer&& other) noexcept : id_{other.id_}, type_(other.type_), usage_(other.usage_) {
  other.id_ = GL_INVALID_INDEX;
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

void Buffer::Bind() {
  AXGL_CALL(glBindBuffer(static_cast<GLenum>(type_), id_));
}

void Buffer::Unbind() {
  AXGL_CALL(glBindBuffer(static_cast<GLenum>(type_), 0));
}

void Buffer::Write(const void* data, size_t size) {
  AXGL_CALL(glBufferData(static_cast<GLenum>(type_), size, data, static_cast<GLenum>(usage_)));
}

void Buffer::WriteSub(const void* data, size_t size, size_t offset) {
  AXGL_CALL(glBufferSubData(static_cast<GLenum>(type_), offset, size, data));
}

}  // namespace ax::gl
