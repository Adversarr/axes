#pragma once
#include <absl/container/flat_hash_map.h>

#include <string>
#include <variant>

#include "axes/core/common.hpp"
#include "axes/utils/status.hpp"
#include "common.hpp"

namespace ax::utils {

class Opt;
using OptValue = std::variant<idx, real, std::string, Opt>;

/**
 * @class Opt
 * @brief A class representing an optional container that stores key-value pairs.
 * 
 * The Opt class provides methods for adding, removing, and accessing key-value pairs.
 * It also supports querying and printing the contents of the container.
 */
class Opt {
public:
  using value_type = OptValue;
  using container_type = absl::flat_hash_map<std::string, value_type>;

  /**
   * @brief Default constructor.
   */
  Opt() = default;

  /**
   * @brief Constructor that initializes the container with a list of key-value pairs.
   * @param init The list of key-value pairs to initialize the container with.
   */
  Opt(std::initializer_list<std::pair<std::string, value_type>> init) : dict_{init} {}
  AX_DECLARE_CONSTRUCTOR(Opt, default, default);

  /**
   * @section Add
   * @brief Methods for adding key-value pairs to the container.
   */

  /**
   * @brief Adds a key-value pair to the container.
   * @param key The key of the pair.
   * @param value The value of the pair.
   * @return A reference to the value associated with the key.
   */
  OptValue& Emplace(const std::string& key, value_type&& value) {
    return dict_.emplace(key, std::move(value)).first->second;
  }

  /**
   * @brief Adds a key-value pair to the container.
   * @param key The key of the pair.
   * @param value The value of the pair.
   * @return A reference to the value associated with the key.
   */
  OptValue& Emplace(const std::string& key, const value_type& value) {
    return dict_.emplace(key, value).first->second;
  }

  /**
   * @section Remove
   * @brief Methods for removing key-value pairs from the container.
   */

  /**
   * @brief Removes a key-value pair from the container.
   * @param key The key of the pair to remove.
   */
  void Erase(const std::string& key) { dict_.erase(key); }

  /**
   * @brief Removes all key-value pairs from the container.
   */
  void Clear() { dict_.clear(); }

  /**
   * @section Access
   * @brief Methods for accessing key-value pairs in the container.
   */

  /**
   * @brief Accesses the value associated with a key.
   * @param key The key to access.
   * @return A reference to the value associated with the key.
   */
  OptValue& operator[](const char* key) { return dict_[key]; }

  /**
   * @brief Accesses the value associated with a key.
   * @param key The key to access.
   * @return A const reference to the value associated with the key.
   */
  OptValue const& operator[](const char* key) const { return dict_.at(key); }

  /**
   * @brief Accesses the value associated with a key.
   * @param key The key to access.
   * @return A reference to the value associated with the key.
   */
  OptValue& operator[](const std::string& key) { return dict_[key]; }

  /**
   * @brief Accesses the value associated with a key.
   * @param key The key to access.
   * @return A const reference to the value associated with the key.
   */
  OptValue const& operator[](const std::string& key) const { return dict_.at(key); }

  /**
   * @section Iterators
   * @brief Methods for iterating over the key-value pairs in the container.
   */

  /**
   * @brief Returns an iterator to the beginning of the container.
   * @return An iterator to the beginning of the container.
   */
  auto begin() { return dict_.begin(); }

  /**
   * @brief Returns an iterator to the end of the container.
   * @return An iterator to the end of the container.
   */
  auto end() { return dict_.end(); }

  /**
   * @brief Returns an iterator to the beginning of the container.
   * @return An iterator to the beginning of the container.
   */
  auto begin() const { return dict_.begin(); }

  /**
   * @brief Returns an iterator to the end of the container.
   * @return An iterator to the end of the container.
   */
  auto end() const { return dict_.end(); }

  /**
   * @section Queriers
   * @brief Methods for querying the container.
   */

  /**
   * @brief Checks if a key exists in the container and if its value is of a specific type.
   * @tparam T The type to check for.
   * @param key The key to check.
   * @return A pair of booleans indicating if the key exists and if its value is of the specified type.
   */
  template <typename T> bool Has(const std::string& key) const { return Check<T>(key).first; }

  /**
   * @brief Checks if a key exists in the container and if its value is holding a specific type.
   * @tparam T The type to check for.
   * @param key The key to check.
   * @return A pair of booleans indicating if the key exists and if its value is holding the specified type.
   */
  template <typename T> bool Holds(const std::string& key) const { return Check<T>(key).second; }

  /**
   * @brief Checks if a key exists in the container and if its value is of a specific type.
   * @tparam T The type to check for.
   * @param key The key to check.
   * @return A pair of booleans indicating if the key exists and if its value is of the specified type.
   */
  template <typename T> std::pair<bool, bool> Check(const std::string& key) const;

  /**
   * @brief Accesses the value associated with a key.
   * @param key The key to access.
   * @return A reference to the value associated with the key.
   */
  OptValue& At(const std::string& key) { return dict_.at(key); }

  /**
   * @brief Accesses the value associated with a key.
   * @param key The key to access.
   * @return A const reference to the value associated with the key.
   */
  OptValue const& At(const std::string& key) const { return dict_.at(key); }

  /**
   * @brief Returns the number of key-value pairs in the container.
   * @return The number of key-value pairs in the container.
   */
  size_t Size() const { return dict_.size(); }

  /**
   * @brief Gets the value associated with a key, or a default value if the key does not exist.
   * @tparam T The type of the value.
   * @param key The key to get the value for.
   * @param default_value The default value to return if the key does not exist.
   * @return The value associated with the key, or the default value if the key does not exist.
   */
  template <typename T> T GetDefault(std::string const& key, T const& default_value) const;

  /**
   * @brief Gets the value associated with a key.
   * @tparam T The type of the value.
   * @param key The key to get the value for.
   * @return The value associated with the key.
   */
  template <typename T> T const& Get(std::string const& key) const;

  /**
   * @section Print
   * @brief Methods for printing the contents of the container.
   */

  /**
   * @brief Prints the contents of the container to an output stream.
   * @param os The output stream to print to.
   * @param depth The depth of the container (used for indentation).
   */
  void Print(std::ostream&, idx depth = 0) const noexcept;

private:
  container_type dict_;
};

std::ostream& operator<<(std::ostream& os, Opt const& opt);

/**
 * @brief The Tunable class represents an interface for objects that can be tuned with options.
 */
class Tunable {
public:

  /**
   * @brief Sets the options for the object.
   * 
   * @param option The options to be set.
   * @return The status of the operation.
   */
  virtual Status SetOptions(utils::Opt const& option);

  /**
   * @brief Gets the current options of the object.
   * 
   * @return The current options.
   */
  virtual utils::Opt GetOptions() const;
};

/**
 * Synchronizes a value with a field in the options object.
 * 
 * This function checks if the specified field exists in the options object and
 * if it holds the correct type. If the field exists and holds the correct type,
 * the value is updated with the field value from the options object.
 * 
 * @tparam T The type of the value to synchronize.
 * @param value The value to synchronize.
 * @param options The options object to synchronize with.
 * @param name The name of the field to synchronize.
 * @return A StatusOr<bool> indicating whether the synchronization was successful.
 *         If the field does not exist or does not hold the correct type, false is returned.
 *         If an error occurs during synchronization, an error status is returned.
 *         If the synchronization is successful, true is returned.
 */
template <typename T> StatusOr<bool> sync_to_field(T& value, Opt const& options, const char* name) {
  using value_t = std::decay_t<T>;
  if constexpr (std::is_same_v<value_t, std::string> || std::is_same_v<value_t, idx>
                || std::is_same_v<value_t, real>) {
    bool has_field = options.Has<T>(name);
    if (!has_field) {
      return false;
    }
    if (!options.Holds<T>(name)) {
      return utils::InvalidArgumentError(std::string("Expected [") + name + std::string("] to be")
                                         + typeid(T).name());
    }
    value = options.Get<T>(name);
  } else {
    bool has_field = options.Has<Opt>(name);
    if (!has_field) {
      return false;
    }
    if (!options.Holds<Opt>(name)) {
      return utils::InvalidArgumentError(std::string("Expected [") + name + std::string("] to be")
                                         + typeid(Opt).name());
    }
    AX_RETURN_NOTOK(value.SetOptions(options.Get<Opt>(name)));
  }
  return true;
}

#define AX_SYNC_OPT(opt, type, var)                                         \
  if (auto status = ::ax::utils::sync_to_field<type>(var##_, opt, #var); !status.ok()) { \
    return status.status();                                                 \
  }

#define AX_SYNC_OPT_IF(opt, type, var) AX_SYNC_OPT(opt, type, var) else if (status.value())

}  // namespace ax::utils
