#pragma once
#include <absl/container/flat_hash_map.h>

#include <string>
#include <variant>

#include "ax/core/common.hpp"
#include "ax/utils/status.hpp"
#include "common.hpp"

#include "enum_refl.hpp"

#include <boost/json/object.hpp>

namespace ax::utils {

using Opt = boost::json::object;

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
template <typename T> struct SyncToFieldHelper {
  StatusOr<bool> Apply(T& value, Opt const& options, const char* name) {
    if (auto it = options.find(name); it != options.end()) {
      if (it->value().is_object()) {
        if (auto status = value.SetOptions(it->value().as_object()); !status.ok()) {
          return status;
        }
        return true;
      } else {
        return utils::InvalidArgumentError("Expect [" + std::string(name)
                                           + "] to be an json-object.");
      }
    } else {
      // return utils::InvalidArgumentError(std::string("Expected [") + name + std::string("] to be")
      //                                   + typeid(T).name());
      return false;
    }
  }
};

template <> struct SyncToFieldHelper<idx> {
  StatusOr<bool> Apply(idx& value, Opt const& options, const char* name) {
    if (auto it = options.find(name); it == options.end()) {
      // return utils::InvalidArgumentError(std::string("Expected [") + name + "] not found");
      return false;
    } else {
      idx old_val = value;
      if (it->value().is_int64()) {
        value = static_cast<idx>(it->value().as_int64());
      } else if (it->value().is_uint64()) {
        value = static_cast<idx>(it->value().as_uint64());
      } else {
        return utils::InvalidArgumentError("Expect [" + std::string(name)
                                           + "] to be a int or uint.");
      }
      return old_val != value;
    }
  }
};

template <> struct SyncToFieldHelper<real> {
  StatusOr<bool> Apply(real& value, Opt const& options, const char* name) {
    if (auto it = options.find(name); it == options.end()) {
      // return utils::InvalidArgumentError(std::string("Expected [") + name + "] not found");
      return false;
    }
    else {
      real old_val = value;
      if (it->value().is_double()) {
        value = static_cast<real>(it->value().as_double());
      } else if (it->value().is_int64()) {
        value = static_cast<real>(it->value().as_int64());
      } else if (it->value().is_uint64()) {
        value = static_cast<real>(it->value().as_uint64());
      } else {
        return utils::InvalidArgumentError("Expect [" + std::string(name)
                                                  + "] to be a double, int or uint.");
      }
      return old_val != value;
    }
  }
};

template <> struct SyncToFieldHelper<std::string> {
  StatusOr<bool> Apply(std::string& value, Opt const& options, const char* name) {
    if (auto it = options.find(name); it == options.end()) {
      // return utils::InvalidArgumentError(std::string("Expected [") + name + "] not found");
      return false;
    }
    else {
      std::string old_val = value;
      if (it->value().is_string()) {
        value = it->value().as_string().c_str();
      } else {
        return utils::InvalidArgumentError("Expect [" + std::string(name)
                                                       + "] to be a string.");
      }
      return old_val != value;
    }
  }
};

template <> struct SyncToFieldHelper<bool> {
  StatusOr<bool> Apply(bool& value, Opt const& options, const char* name) {
    if (auto it = options.find(name); it == options.end()) {
      // return utils::InvalidArgumentError(std::string("Expected [") + name + "] not found");
      return false;
    } else {
      bool old_val = value;
      if (it->value().is_bool()) {
        value = it->value().as_bool();
      } else if (it->value().is_int64()) {
        value = it->value().as_int64() != 0;
      } else if (it->value().is_uint64()) {
        value = it->value().as_uint64() != 0;
      } else {
        return utils::InvalidArgumentError("Expect [" + std::string(name)
                                                        + "] to be a bool.");
      }
      return old_val != value;
    }
  }
};


template <typename T>
StatusOr<bool> sync_to_field(T& value, Opt const& options, const char* name) {
  return SyncToFieldHelper<std::decay_t<T>>{}.Apply(value, options, name);
}

#define AX_SYNC_OPT(opt, type, var)                                         \
  if (auto status = ::ax::utils::sync_to_field<type>(var##_, opt, #var); !status.ok()) { \
    return status.status();                                                 \
  }

#define AX_SYNC_OPT_IF(opt, type, var) AX_SYNC_OPT(opt, type, var) else if (status.value())

}  // namespace ax::utils
