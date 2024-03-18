#pragma once

#include "axes/core/entt.hpp"
namespace ax::cmpt {

/**
 * @brief The Name struct represents the name of an entity.
 */
struct Name {
  std::string value_; /**< The value of the name. */
};

/**
 * @brief Creates a named entity with the specified name.
 * 
 * @param name The name of the entity.
 * @return The created entity.
 */
Entity create_named_entity(const std::string& name);

/**
 * @brief Retrieves the name of the specified entity.
 * 
 * @param entity The entity.
 * @return The name of the entity, if it has a name; otherwise, std::nullopt.
 */
std::optional<std::string> get_entity_name(Entity entity);

}