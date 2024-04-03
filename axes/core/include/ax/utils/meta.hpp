#pragma once
#include <typeindex>
#include <string>


namespace ax::utils {

/**
 * @brief The Identifier for Type.
 *
 */
struct TypeMeta {
  std::type_index index_;
  std::string name_;
};

namespace details {
void initialize_type_meta_registry();
}  // namespace details

TypeMeta const* get_type_meta(std::type_index ti);

/**
  * @brief Register the type meta.
  * @param meta The type meta to register.
  * @note This function is not thread safe. And, although this method can be called multiple times,
  *       it is not recommended to call this method multiple times.
  */
TypeMeta const* register_type_meta(TypeMeta const& meta);



}  // namespace ax::graph
