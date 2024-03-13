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

class Opt {
public:
  using value_type = OptValue;
  using container_type = absl::flat_hash_map<std::string, value_type>;

  Opt() = default;

  Opt(std::initializer_list<std::pair<std::string, value_type>> init) : dict_{init} {}
  AX_DECLARE_CONSTRUCTOR(Opt, default, default);

  /****************************** SECT: Add ******************************/
  OptValue& Emplace(const std::string& key, value_type&& value) {
    return dict_.emplace(key, std::move(value)).first->second;
  }

  OptValue& Emplace(const std::string& key, const value_type& value) {
    return dict_.emplace(key, value).first->second;
  }

  /****************************** SECT: Remove ******************************/
  void Erase(const std::string& key) { dict_.erase(key); }

  void Clear() { dict_.clear(); }

  /****************************** SECT: Access ******************************/
  OptValue& operator[](const char* key) { return dict_[key]; }

  OptValue const& operator[](const char* key) const { return dict_.at(key); }

  OptValue& operator[](const std::string& key) { return dict_[key]; }

  OptValue const& operator[](const std::string& key) const { return dict_.at(key); }

  /****************************** SECT: Iterators ******************************/

  auto begin() { return dict_.begin(); }

  auto end() { return dict_.end(); }

  auto begin() const { return dict_.begin(); }

  auto end() const { return dict_.end(); }

  /****************************** Queriers ******************************/

  template <typename T> bool Has(const std::string& key) const { return Check<T>(key).first; }

  template <typename T> bool Holds(const std::string& key) const { return Check<T>(key).second; }

  template <typename T> std::pair<bool, bool> Check(const std::string& key) const;

  OptValue& At(const std::string& key) { return dict_.at(key); }

  OptValue const& At(const std::string& key) const { return dict_.at(key); }

  size_t Size() const { return dict_.size(); }

  template <typename T> T GetDefault(std::string const& key, T const& default_value) const;

  template <typename T> T const& Get(std::string const& key) const;

  /****************************** Print ******************************/
  void Print(std::ostream&, idx depth = 0) const noexcept;

private:
  container_type dict_;
};

std::ostream& operator<<(std::ostream& os, Opt const& opt);

class Tunable {
public:
  virtual Status SetOptions(utils::Opt const& option);
  virtual utils::Opt GetOptions() const;
};

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
