#pragma once
#include <fmt/ostream.h>

#include <boost/json/object.hpp>
#include <string>

#include "ax/core/common.hpp"
#include "ax/core/excepts.hpp"
#include "ax/utils/enum_refl.hpp"

namespace fmt {
template <> struct formatter<boost::json::string> : ostream_formatter {};
}  // namespace fmt

namespace ax::utils {

using Options = boost::json::object;

/**
 * @brief The Tunable class represents an interface for objects that can be tuned with options.
 */
class Tunable {
public:
  Tunable() noexcept = default;
  virtual ~Tunable() noexcept = default;

  /**
   * @brief Sets the options for the object.
   *
   * @param option The options to be set.
   * @return The status of the operation.
   */
  virtual void SetOptions(utils::Options const& option);

  /**
   * @brief Gets the current options of the object.
   *
   * @return The current options.
   */
  AX_NODISCARD virtual utils::Options GetOptions() const;
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
  bool Apply(T& value, Options const& options, const char* name) {
    if (auto it = options.find(name); it != options.end()) {
      if (it->value().is_object()) {
        value.SetOptions(it->value().as_object());
        return true;
      } else {
        throw std::invalid_argument("Expect [" + std::string(name) + "] to be an json-object.");
      }
    } else {
      return false;
    }
  }
};

template <> struct SyncToFieldHelper<Index> {
  bool Apply(Index& value, Options const& options, const char* name) {
    if (auto it = options.find(name); it == options.end()) {
      return false;
    } else {
      Index old_val = value;
      if (it->value().is_int64()) {
        value = static_cast<Index>(it->value().as_int64());
      } else if (it->value().is_uint64()) {
        value = static_cast<Index>(it->value().as_uint64());
      } else {
        throw std::invalid_argument("Expect [" + std::string(name) + "] to be a int or uint.");
      }
      return old_val != value;
    }
  }
};

template <> struct SyncToFieldHelper<real> {
  bool Apply(real& value, Options const& options, const char* name) {
    if (auto it = options.find(name); it == options.end()) {
      return false;
    } else {
      real old_val = value;
      if (it->value().is_double()) {
        value = static_cast<real>(it->value().as_double());
      } else if (it->value().is_int64()) {
        value = static_cast<real>(it->value().as_int64());
      } else if (it->value().is_uint64()) {
        value = static_cast<real>(it->value().as_uint64());
      } else {
        throw std::invalid_argument("Expect [" + std::string(name)
                                    + "] to be a double, int or uint.");
      }
      return old_val != value;
    }
  }
};

template <> struct SyncToFieldHelper<std::string> {
  bool Apply(std::string& value, Options const& options, const char* name) {
    if (auto it = options.find(name); it == options.end()) {
      // return utils::InvalidArgumentError(std::string("Expected [") + name + "] not found");
      return false;
    } else {
      std::string old_val = value;
      if (it->value().is_string()) {
        value = it->value().as_string().c_str();
      } else {
        // return utils::InvalidArgumentError("Expect [" + std::string(name)
        //                                                + "] to be a string.");
        throw std::invalid_argument("Expect [" + std::string(name) + "] to be a string.");
      }
      return old_val != value;
    }
  }
};

template <> struct SyncToFieldHelper<bool> {
  bool Apply(bool& value, Options const& options, const char* name) {
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
        throw std::invalid_argument("Expect [" + std::string(name) + "] to be a bool.");
      }
      return old_val != value;
    }
  }
};

template <typename T> bool sync_from_opt(T& value, Options const& options, const char* name) {
  return SyncToFieldHelper<std::decay_t<T>>{}.Apply(value, options, name);
}

inline const char* extract_string(boost::json::value const& v) {
  AX_THROW_IF_FALSE(v.is_string(), "Expect a string.");
  return v.as_string().c_str();
}

inline bool extract_bool(boost::json::value const& v) {
  AX_THROW_IF_FALSE(v.is_bool(), "Expect a bool.");
  return v.as_bool();
}

inline Index extract_Index(boost::json::value const& v) {
  AX_THROW_IF_FALSE(v.is_int64(), "Expect a int64.");
  return v.as_int64();
}

inline real extract_real(boost::json::value const& v) {
  AX_THROW_IF_FALSE(v.is_double() || v.is_int64() || v.is_uint64(),
                    "Expect a double, int64 or uint64.");
  if (v.is_double()) {
    return v.as_double();
  } else if (v.is_int64()) {
    return static_cast<real>(v.as_int64());
  } else if (v.is_uint64()) {
    return static_cast<real>(v.as_uint64());
  }
  AX_UNREACHABLE();
}

template <typename Enum>
std::pair<bool, std::optional<Enum>> extract_enum(utils::Options const& opt, const char* name) {
  if (auto it = opt.find(name); it != opt.end()) {
    AX_THROW_IF_FALSE(it->value().is_string(), "Expect '{}' to be a string", std::string(name));
    return {true, utils::reflect_enum<Enum>(it->value().as_string().c_str())};
  } else {
    return {false, std::nullopt};
  }
}

template <typename AnyTunable>
bool extract_tunable(utils::Options const& opt, const char* key, AnyTunable* tun) {
  static_assert(std::is_base_of_v<Tunable, AnyTunable>, "Only Tunable is supported");
  if (tun == nullptr) {
    return false;
  }
  if (auto it = opt.find(key); it != opt.end()) {
    AX_THROW_IF_FALSE(it->value().is_object(), "Expect '{}' to be a json object", std::string(key));
    tun->SetOptions(it->value().as_object());
    return true;
  } else {
    return false;
  }
}

template <typename Factory, typename Kind>
auto extract_and_create(utils::Options const& opt, const char* key) {
  auto [has_key, kind] = utils::extract_enum<Kind>(opt, key);
  if (has_key) {
    if (kind) {
      return Factory::Create(kind.value());
    } else {
      throw make_invalid_argument("Invalid {} option: {}", key, opt.at(key).as_string());
    }
  }
}

template <typename Factory, typename Kind, typename Ptr>
bool extract_and_create(utils::Options const& opt, const char* key, Ptr& ptr) {
  auto [has_key, kind] = utils::extract_enum<Kind>(opt, key);
  if (has_key) {
    if (kind) {
      ptr = Factory::Create(kind.value());
      return true;
    } else {
      throw make_invalid_argument("Invalid \"{}\" option: {}", key, opt.at(key).as_string());
    }
  }
  return false;
}

template <typename Kind> bool extract_enum(utils::Options const& opt, const char* key, Kind& kind) {
  auto [has_key, _kind] = utils::extract_enum<Kind>(opt, key);
  if (has_key) {
    if (_kind) {
      kind = _kind.value();
      return true;
    } else {
      throw make_invalid_argument("Invalid \"{}\" option: {}", key,
                                  std::string(opt.at(key).as_string()));
    }
  }
  return false;
}

template <typename Kind> auto extract_enum_force(utils::Options const& opt, const char* key) {
  auto [has_key, kind] = utils::extract_enum<Kind>(opt, key);
  if (has_key) {
    if (kind) {
      return kind.value();
    } else {
      throw make_invalid_argument("Invalid \"{}\" option: {}", key, opt.at(key).as_string());
    }
  }
}

#define AX_SYNC_OPT(opt, type, var) ax::utils::sync_from_opt<type>(var##_, (opt), #var)

#define AX_SYNC_OPT_IF(opt, type, var) if (AX_SYNC_OPT(opt, type, var))

#define AX_SYNC_OPT_FACTORY(opt, type, var, name, factory)                         \
  do {                                                                             \
    auto [has_##name, _kind_##name] = ax::utils::extract_enum<type>((opt), #name); \
    if (has_##name) {                                                              \
      if (_kind_##name) {                                                          \
        (var) = factory::Create(_kind_##name.value());                             \
      } else {                                                                     \
        throw make_invalid_argument("Invalid \"{}\" option: {}", #name,            \
                                    (opt).at(#name).as_string());                  \
      }                                                                            \
    }                                                                              \
  } while (false)

#define AX_SYNC_OPT_ENUM(opt, type, var, name)                                     \
  do {                                                                             \
    auto [has_##name, _enum_##name] = ax::utils::extract_enum<type>((opt), #name); \
    if (has_##name) {                                                              \
      if (_enum_##name) {                                                          \
        (var) = (_enum_##name).value();                                            \
      } else {                                                                     \
        throw make_invalid_argument("Invalid \"{}\" option: {}", #name,            \
                                    (opt).at(#name).as_string());                  \
      }                                                                            \
    }                                                                              \
  } while (false)

}  // namespace ax::utils
