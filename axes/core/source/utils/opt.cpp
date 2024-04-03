#include "ax/utils/opt.hpp"

#include "ax/core/echo.hpp"

namespace ax::utils {

void Opt::Print(std::ostream& os, idx depth) const noexcept {
  os << "{";
  for (auto it = dict_.begin(); it != dict_.end(); ++it) {
    if (it != dict_.begin()) {
      os << ", " << std::endl;
    } else {
      os << std::endl;
    }
    os << std::string(2 * (depth + 1), ' ') << it->first << ": ";
    std::visit(
        [&os, depth](auto&& arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, std::string>) {
            os << std::quoted(arg);
          } else if constexpr (std::is_same_v<T, idx>) {
            os << arg;
          } else if constexpr (std::is_same_v<T, real>) {
            os << arg;
          } else if constexpr (std::is_same_v<T, Opt>) {
            arg.Print(os, depth + 1);
          } else {
            AX_CHECK(false) << "Unsupported type: " << typeid(T).name();
          }
        },
        it->second);
  }
  if (!dict_.empty()) {
    os << std::endl << std::string(2 * depth, ' ');
  }
  os << "}";
}

std::ostream& operator<<(std::ostream& os, Opt const& opt) {
  opt.Print(os);
  return os;
}

/************************* SECT: Get *************************/
template <typename T> T const& GetImpl(Opt::container_type const& dict, std::string const& key) {
  return std::get<T>(dict.at(key));
}

template <> idx const& Opt::Get<idx>(std::string const& key) const {
  return GetImpl<idx>(dict_, key);
}
template <> real const& Opt::Get<real>(std::string const& key) const {
  return GetImpl<real>(dict_, key);
}
template <> std::string const& Opt::Get<std::string>(std::string const& key) const {
  return GetImpl<std::string>(dict_, key);
}
template <> Opt const& Opt::Get<Opt>(std::string const& key) const {
  return GetImpl<Opt>(dict_, key);
}

/************************* SECT: Get Default *************************/
template <typename T>
T GetDefaultImpl(Opt::container_type const& dict, std::string const& key, T const& default_value) {
  if (auto it = dict.find(key); it != dict.end()) {
    return std::get<T>(it->second);
  } else {
    return default_value;
  }
}

template <> idx Opt::GetDefault<idx>(std::string const& key, idx const& default_value) const {
  return GetDefaultImpl<idx>(dict_, key, default_value);
}
template <> real Opt::GetDefault<real>(std::string const& key, real const& default_value) const {
  return GetDefaultImpl<real>(dict_, key, default_value);
}
template <> std::string Opt::GetDefault<std::string>(std::string const& key,
                                                     std::string const& default_value) const {
  return GetDefaultImpl<std::string>(dict_, key, default_value);
}
template <> Opt Opt::GetDefault<Opt>(std::string const& key, Opt const& default_value) const {
  return GetDefaultImpl<Opt>(dict_, key, default_value);
}

/************************* SECT: Check *************************/
template <typename T>
std::pair<bool, bool> CheckImpl(Opt::container_type const& dict, const std::string& key) {
  if (auto it = dict.find(key); it != dict.end()) {
    return {true, std::holds_alternative<T>(it->second)};
  } else {
    return {false, false};
  }
}

template <> std::pair<bool, bool> Opt::Check<idx>(const std::string& key) const {
  return CheckImpl<idx>(dict_, key);
}
template <> std::pair<bool, bool> Opt::Check<real>(const std::string& key) const {
  return CheckImpl<real>(dict_, key);
}
template <> std::pair<bool, bool> Opt::Check<std::string>(const std::string& key) const {
  return CheckImpl<std::string>(dict_, key);
}
template <> std::pair<bool, bool> Opt::Check<Opt>(const std::string& key) const {
  return CheckImpl<Opt>(dict_, key);
}

utils::Opt Tunable::GetOptions() const { return {}; }
Status Tunable::SetOptions(utils::Opt const&) { AX_RETURN_OK(); }

}  // namespace ax::utils
