//
// Created by Yang Jerry on 2023/5/9.
//
#pragma once
#include <memory>
#include <vector>

namespace axes::ecs::details {

/**
 * @brief Memory chunk for ecs system.
 *
 * @tparam Type
 * @tparam chunk_size
 * @tparam Allocator
 */
template <typename Type, size_t chunk_size = 32> class Chunk {
public:
  explicit Chunk();

  ~Chunk();

  Chunk(const Chunk &) = delete;

  Chunk(Chunk &&from);

  template <typename... Args> [[nodiscard]] Type *Create(Args &&...val);

  Type *At(size_t n) noexcept { return data_ + n; }

  Type *operator[](size_t d) noexcept { return data_ + d; }

  const Type *RawData() noexcept { return data_; }

  void Destroy(Type *ptr);

  bool IsFull() const noexcept;

  size_t Request();

private:
  // Sparse set:
  std::array<size_t, chunk_size> constructed_memories_;
  std::array<size_t, chunk_size> reversed_id_map_;
  size_t used_size_{0};
  Type *data_;
};

/******************** IMPL ********************/

template <typename Type, size_t chunk_size> Chunk<Type, chunk_size>::~Chunk() {
  if (data_ == nullptr) {
    return;
  }
  // Destroy active objects.
  for (size_t i = 0; i < used_size_; ++i) {
    std::destroy_at(data_ + constructed_memories_[i]);
  }
  free(data_);
  data_ = nullptr;
}

template <typename Type, size_t chunk_size>
Chunk<Type, chunk_size>::Chunk(Chunk &&from) {
  data_ = from.data_;
  constructed_memories_ = from.constructed_memories_;
  reversed_id_map_ = from.reversed_id_map_;
  used_size_ = from.used_size_;
  from.data_ = nullptr;
}

template <typename Type, size_t chunk_size> template <typename... Args>
Type *Chunk<Type, chunk_size>::Create(Args &&...val) {
  size_t avail_mem_id = Request();
  Type *avail_mem = data_ + avail_mem_id;
  std::construct_at(avail_mem, std::forward<Args>(val)...);
  return avail_mem;
}

template <typename Type, size_t chunk_size>
void Chunk<Type, chunk_size>::Destroy(Type *ptr) {
  std::ptrdiff_t diff = ptr - data_;
  if (diff < 0 || static_cast<size_t>(diff) > chunk_size) {
    throw std::out_of_range("Destroy out of range.");
  }
  // Find the index in constructed memory.
  auto ci = reversed_id_map_[diff];
  --used_size_;
  // swap `ci` and `used_size`, and delete used_size.
  std::swap(constructed_memories_[ci], constructed_memories_[used_size_]);
  std::swap(reversed_id_map_[constructed_memories_[ci]],
            reversed_id_map_[constructed_memories_[used_size_]]);
  std::destroy_at(ptr);
}

template <typename Type, size_t chunk_size>
bool Chunk<Type, chunk_size>::IsFull() const noexcept {
  return used_size_ == chunk_size;
}

template <typename Type, size_t chunk_size>
size_t Chunk<Type, chunk_size>::Request() {
  if (IsFull()) [[unlikely]] {
    throw std::out_of_range("Chunk exhausted.");
  }
  size_t free_mem_id = constructed_memories_[used_size_];
  ++used_size_;
  return free_mem_id;
}

template <typename Type, size_t chunk_size> Chunk<Type, chunk_size>::Chunk()
    : used_size_{0} {
  for (size_t i = 0; i < chunk_size; ++i) {
    constructed_memories_[i] = i;
    reversed_id_map_[i] = i;
  }
  data_ = (Type* ) malloc(sizeof(Type) * chunk_size);
}
}  // namespace axes::ecs::details
