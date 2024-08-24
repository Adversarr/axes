#pragma once
#include <fmt/ostream.h>

#include <boost/json/object.hpp>
#include <boost/json/serialize.hpp>
#include <string>

namespace ax::utils {
inline void json_pretty_print(std::ostream& os, boost::json::value const& jv,
                              std::string* indent_ptr = nullptr) {
  std::string indent_cur;
  if (!indent_ptr)
    indent_ptr = &indent_cur;
  switch (jv.kind()) {
    case boost::json::kind::object: {
      auto const& obj = jv.get_object();
      if (obj.empty()) {
        os << "{}";
      } else {
        os << "{\n";
        indent_ptr->append(4, ' ');
        auto it = obj.begin();
        for (;;) {
          os << *indent_ptr << boost::json::serialize(it->key()) << " : ";
          json_pretty_print(os, it->value(), indent_ptr);
          if (++it == obj.end())
            break;
          os << ",\n";
        }
        os << "\n";
        indent_ptr->resize(indent_ptr->size() - 4);
        os << *indent_ptr << "}";
      }
      break;
    }

    case boost::json::kind::array: {
      auto const& arr = jv.get_array();
      if (arr.empty()) {
        os << "[]";
      } else {
        os << "[\n";
        indent_ptr->append(4, ' ');
        if (!arr.empty()) {
          auto it = arr.begin();
          for (;;) {
            os << *indent_ptr;
            json_pretty_print(os, *it, indent_ptr);
            if (++it == arr.end())
              break;
            os << ",\n";
          }
        }
        os << "\n";
        indent_ptr->resize(indent_ptr->size() - 4);
        os << *indent_ptr << "]";
      }
      break;
    }

    case boost::json::kind::string: {
      os << boost::json::serialize(jv.get_string());
      break;
    }

    case boost::json::kind::uint64:
      os << jv.get_uint64();
      break;

    case boost::json::kind::int64:
      os << jv.get_int64();
      break;

    case boost::json::kind::double_:
      os << jv.get_double();
      break;

    case boost::json::kind::bool_:
      if (jv.get_bool())
        os << "true";
      else
        os << "false";
      break;

    case boost::json::kind::null:
      os << "null";
      break;
  }

  if (indent_ptr->empty())
    os << "\n";
}

}  // namespace ax::utils

namespace fmt {
template <>
struct formatter<boost::json::value> {
  constexpr auto parse(format_parse_context& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(boost::json::value const& jv, FormatContext& ctx) const {
    std::ostringstream oss;
    ax::utils::json_pretty_print(oss, jv);
    return format_to(ctx.out(), "{}", oss.str());
  }
};

template <>
struct formatter<boost::json::object> {
  constexpr auto parse(format_parse_context& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(boost::json::object const& jv, FormatContext& ctx) const {
    std::ostringstream oss;
    ax::utils::json_pretty_print(oss, boost::json::value(jv));
    return format_to(ctx.out(), "{}", oss.str());
  }
};

template <>
struct formatter<boost::json::array> {
  constexpr auto parse(format_parse_context& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(boost::json::array const& jv, FormatContext& ctx) const {
    std::ostringstream oss;
    ax::utils::json_pretty_print(oss, boost::json::value(jv));
    return format_to(ctx.out(), "{}", oss.str());
  }
};

}  // namespace fmt