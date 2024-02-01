#pragma once
#include <string>
#include <unordered_map>
#include <variant>

#include "axes/core/common.hpp"
#include "common.hpp"

namespace ax::utils {

class Opt;
using OptValue = std::variant<idx, real, std::string, Opt>;

class Opt {
public:
  using value_type = OptValue;
  using container_type = std::unordered_map<std::string, value_type>;

  Opt() = default;
  AX_DECLARE_CONSTRUCTOR(Opt, default, default);

  /****************************** Add ******************************/
  OptValue& Emplace(const std::string& key, value_type&& value) {
    return dict_.emplace(key, std::move(value)).first->second;
  }

  OptValue& Emplace(const std::string& key, const value_type& value) {
    return dict_.emplace(key, value).first->second;
  }

  /****************************** Remove ******************************/

  void Erase(const std::string& key) { dict_.erase(key); }

  void Clear() { dict_.clear(); }

  /****************************** Access ******************************/

  OptValue& operator[](const char* key) { return dict_[key]; }

  OptValue const& operator[](const char* key) const { return dict_.at(key); }

  OptValue& operator[](const std::string& key) { return dict_[key]; }

  OptValue const& operator[](const std::string& key) const { return dict_.at(key); }

  /****************************** Iterators ******************************/

  auto begin() { return dict_.begin(); }

  auto end() { return dict_.end(); }

  auto begin() const { return dict_.begin(); }

  auto end() const { return dict_.end(); }

  /****************************** Queriers ******************************/

  bool Has(const std::string& key) const { return dict_.find(key) != dict_.end(); }

  OptValue& At(const std::string& key) { return dict_.at(key); }

  OptValue const& At(const std::string& key) const { return dict_.at(key); }

  size_t Size() const { return dict_.size(); }

  /****************************** Print ******************************/

  void Print(std::ostream&, idx depth = 0) const noexcept;

private:
  container_type dict_;
};

}  // namespace ax::utils
