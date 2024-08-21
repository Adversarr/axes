// The MIT License (MIT)
// Copyright © 2024 Adversarr
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the “Software”), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// NOTE: Possible control macros are
//   CG_AUTO_REGISTER:--------------------------------------------------------+
//   |  Controls whether to automatically register nodes before main().       |
//   |  Define this macro to enable auto registration, used in `node.hpp`.    |
//   +------------------------------------------------------------------------+
//   CG_NO_STRONG_INLINE:-----------------------------------------------------+
//   |  controls whether to use __forceinline or not, see below.              |
//   |  Define this macro to disable __forceinline, otherwise just `inline`.  |
//   +------------------------------------------------------------------------+
//   CG_NODE_SUPERBASE:-------------------------------------------------------+
//   |  A control macro that can be defined to add extra inheritance to the   |
//   |  node base.                                                            |
//   +------------------------------------------------------------------------+
//   CG_NODE_EXTENSION:-------------------------------------------------------+
//   |  A control macro that can be defined to add extra members to the node  |
//   |  class. (public)                                                       |
//   +------------------------------------------------------------------------+
//   CG_NO_EXCEPTION:---------------------------------------------------------+
//   |  Controls whether to use exceptions or not.                            |
//   |  Define this macro to disable exceptions, and check failures will      |
//   |  fallback to abort().                                                  |
//   +------------------------------------------------------------------------+
//   CG_NO_CHECK:-------------------------------------------------------------+
//   |  Controls whether to use runtime checks or not.                        |
//   |  Define this macro to disable runtime checks, and check failures will  |
//   |  fallback to undefined behavior.                                       |
//   +------------------------------------------------------------------------+
//   CG_SMALL_VECTOR:---------------------------------------------------------+
//   |  Controls whether to use small vector or not.                          |
//   +------------------------------------------------------------------------+

#pragma once

#include <algorithm>
#include <any>
#include <cstddef>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <utility>
#include <variant>
#include <vector>

#ifdef CG_NO_STRONG_INLINE
#  define CG_STRONG_INLINE inline
#else
#  ifdef _MSC_VER
#    define CG_STRONG_INLINE __forceinline
#  elif defined(__GNUC__)
#    define CG_STRONG_INLINE inline __attribute__((always_inline))
#  elif defined(__clang__)
#    define CG_STRONG_INLINE inline __attribute__((always_inline))
#  else
#    define CG_STRONG_INLINE inline
#  endif
#endif

#ifdef CG_NODE_SUPERBASE
#  define CG_NODE_INHERITANCE : CG_NODE_SUPERBASE
#else
#  define CG_NODE_INHERITANCE
#endif

#ifndef CG_NODE_EXTENSION
#  define CG_NODE_EXTENSION /* empty */
#endif

#ifdef CG_NO_CHECK
#  define CG_THROW(type, ...) /* empty */
#  define CG_NOEXCEPT noexcept
#else
#  ifdef CG_NO_EXCEPTION
#    define CG_THROW(type, ...) abort()
#    define CG_NOEXCEPT noexcept
#  else
#include <stdexcept>
#    define CG_THROW(type, ...) throw type(__VA_ARGS__)
#    define CG_NOEXCEPT
#  endif
#endif

#ifdef __clang__
#  define CG_UNREACHABLE() __builtin_unreachable()
#elif defined(__GNUC__)
#  define CG_UNREACHABLE() __builtin_unreachable()
#elif defined(_MSC_VER)
#  define CG_UNREACHABLE() __assume(0)
#else
#  define CG_UNREACHABLE() abort()
#endif

#if __cplusplus >= 201703L
#  define CG_CONSTEXPR constexpr
#else
#  define CG_CONSTEXPR inline
#endif

#ifndef CG_SMALL_VECTOR
#define CG_SMALL_VECTOR(T) std::vector< T >
#endif


#define CG_PP_THIRD_ARG(a,b,c,...) c
#define CG_VA_OPT_SUPPORTED_I(...) CG_PP_THIRD_ARG(__VA_OPT__(,),true,false,)
#define CG_VA_OPT_SUPPORTED CG_VA_OPT_SUPPORTED_I(?)
#if !CG_VA_OPT_SUPPORTED
#  error "VA_OPT is not supported by this compiler"
#endif


#define CG_PP_VAOPT_SIZE(...) CG_PP_VAOPT_SIZE_I(toooo_many, ##__VA_ARGS__, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define CG_PP_VAOPT_SIZE_I(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, N, ...) N

#define CG_PP_VAOPT_FOR_EACH_I_0(macro, ...)
#define CG_PP_VAOPT_FOR_EACH_I_1(macro, _0) macro(_0, 0)
#define CG_PP_VAOPT_FOR_EACH_I_2(macro, _0, _1) CG_PP_VAOPT_FOR_EACH_I_1(macro, _0) macro(_1, 1)
#define CG_PP_VAOPT_FOR_EACH_I_3(macro, _0, _1, _2) CG_PP_VAOPT_FOR_EACH_I_2(macro, _0, _1) macro(_2, 2)
#define CG_PP_VAOPT_FOR_EACH_I_4(macro, _0, _1, _2, _3) CG_PP_VAOPT_FOR_EACH_I_3(macro, _0, _1, _2) macro(_3, 3)
#define CG_PP_VAOPT_FOR_EACH_I_5(macro, _0, _1, _2, _3, _4) CG_PP_VAOPT_FOR_EACH_I_4(macro, _0, _1, _2, _3) macro(_4, 4)
#define CG_PP_VAOPT_FOR_EACH_I_6(macro, _0, _1, _2, _3, _4, _5) CG_PP_VAOPT_FOR_EACH_I_5(macro, _0, _1, _2, _3, _4) macro(_5, 5)
#define CG_PP_VAOPT_FOR_EACH_I_7(macro, _0, _1, _2, _3, _4, _5, _6) CG_PP_VAOPT_FOR_EACH_I_6(macro, _0, _1, _2, _3, _4, _5) macro(_6, 6)
#define CG_PP_VAOPT_FOR_EACH_I_8(macro, _0, _1, _2, _3, _4, _5, _6, _7) CG_PP_VAOPT_FOR_EACH_I_7(macro, _0, _1, _2, _3, _4, _5, _6) macro(_7, 7)
#define CG_PP_VAOPT_FOR_EACH_I_9(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8) CG_PP_VAOPT_FOR_EACH_I_8(macro, _0, _1, _2, _3, _4, _5, _6, _7) macro(_8, 8)
#define CG_PP_VAOPT_FOR_EACH_I_10(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9) CG_PP_VAOPT_FOR_EACH_I_9(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8) macro(_9, 9)
#define CG_PP_VAOPT_FOR_EACH_I_11(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10) CG_PP_VAOPT_FOR_EACH_I_10(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9) macro(_10, 10)
#define CG_PP_VAOPT_FOR_EACH_I_12(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11) CG_PP_VAOPT_FOR_EACH_I_11(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10) macro(_11, 11)
#define CG_PP_VAOPT_FOR_EACH_I_13(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12) CG_PP_VAOPT_FOR_EACH_I_12(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11) macro(_12, 12)
#define CG_PP_VAOPT_FOR_EACH_I_14(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13) CG_PP_VAOPT_FOR_EACH_I_13(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12) macro(_13, 13)
#define CG_PP_VAOPT_FOR_EACH_I_15(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14) CG_PP_VAOPT_FOR_EACH_I_14(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13) macro(_14, 14)
#define CG_PP_VAOPT_FOR_EACH_I_16(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15) CG_PP_VAOPT_FOR_EACH_I_15(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14) macro(_15, 15)
#define CG_PP_VAOPT_FOR_EACH_I_17(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16) CG_PP_VAOPT_FOR_EACH_I_16(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15) macro(_16, 16)
#define CG_PP_VAOPT_FOR_EACH_I_18(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17) CG_PP_VAOPT_FOR_EACH_I_17(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16) macro(_17, 17)
#define CG_PP_VAOPT_FOR_EACH_I_19(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18) CG_PP_VAOPT_FOR_EACH_I_18(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17) macro(_18, 18)
#define CG_PP_VAOPT_FOR_EACH_I_20(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19) CG_PP_VAOPT_FOR_EACH_I_19(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18) macro(_19, 19)
#define CG_PP_VAOPT_FOR_EACH_I_21(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20) CG_PP_VAOPT_FOR_EACH_I_20(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19) macro(_20, 20)
#define CG_PP_VAOPT_FOR_EACH_I_22(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21) CG_PP_VAOPT_FOR_EACH_I_21(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20) macro(_21, 21)
#define CG_PP_VAOPT_FOR_EACH_I_23(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22) CG_PP_VAOPT_FOR_EACH_I_22(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21) macro(_22, 22)
#define CG_PP_VAOPT_FOR_EACH_I_24(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23) CG_PP_VAOPT_FOR_EACH_I_23(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22) macro(_23, 23)
#define CG_PP_VAOPT_FOR_EACH_I_25(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24) CG_PP_VAOPT_FOR_EACH_I_24(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23) macro(_24, 24)
#define CG_PP_VAOPT_FOR_EACH_I_26(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25) CG_PP_VAOPT_FOR_EACH_I_25(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24) macro(_25, 25)
#define CG_PP_VAOPT_FOR_EACH_I_27(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26) CG_PP_VAOPT_FOR_EACH_I_26(macro, _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25) macro(_26, 26)


#define CG_PP_NTH_ARG(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, N, ...) N

#define CG_PP_VAOPT_FOR_EACH_I(macro, ...)  \
    CG_PP_NTH_ARG(~, ##__VA_ARGS__,  \
        CG_PP_VAOPT_FOR_EACH_I_27,          \
        CG_PP_VAOPT_FOR_EACH_I_26,          \
        CG_PP_VAOPT_FOR_EACH_I_25,          \
        CG_PP_VAOPT_FOR_EACH_I_24,          \
        CG_PP_VAOPT_FOR_EACH_I_23,          \
        CG_PP_VAOPT_FOR_EACH_I_22,          \
        CG_PP_VAOPT_FOR_EACH_I_21,          \
        CG_PP_VAOPT_FOR_EACH_I_20,          \
        CG_PP_VAOPT_FOR_EACH_I_19,          \
        CG_PP_VAOPT_FOR_EACH_I_18,          \
        CG_PP_VAOPT_FOR_EACH_I_17,          \
        CG_PP_VAOPT_FOR_EACH_I_16,          \
        CG_PP_VAOPT_FOR_EACH_I_15,          \
        CG_PP_VAOPT_FOR_EACH_I_14,          \
        CG_PP_VAOPT_FOR_EACH_I_13,          \
        CG_PP_VAOPT_FOR_EACH_I_12,          \
        CG_PP_VAOPT_FOR_EACH_I_11,          \
        CG_PP_VAOPT_FOR_EACH_I_10,          \
        CG_PP_VAOPT_FOR_EACH_I_9,           \
        CG_PP_VAOPT_FOR_EACH_I_8,           \
        CG_PP_VAOPT_FOR_EACH_I_7,           \
        CG_PP_VAOPT_FOR_EACH_I_6,           \
        CG_PP_VAOPT_FOR_EACH_I_5,           \
        CG_PP_VAOPT_FOR_EACH_I_4,           \
        CG_PP_VAOPT_FOR_EACH_I_3,           \
        CG_PP_VAOPT_FOR_EACH_I_2,           \
        CG_PP_VAOPT_FOR_EACH_I_1,           \
        CG_PP_VAOPT_FOR_EACH_I_0) (macro, ##__VA_ARGS__)

#define CG_PP_TUPLE_UNPACK(...) __VA_ARGS__

#define CG_PP_EMPTY()
#define CG_PP_DEFER(id) id CG_PP_EMPTY()
#define CG_PP_EVAL(...) CG_PP_EVAL1(CG_PP_EVAL1(CG_PP_EVAL1(__VA_ARGS__)))
#define CG_PP_EVAL1(...) CG_PP_EVAL2(CG_PP_EVAL2(CG_PP_EVAL2(__VA_ARGS__)))
#define CG_PP_EVAL2(...) CG_PP_EVAL3(CG_PP_EVAL3(CG_PP_EVAL3(__VA_ARGS__)))
#define CG_PP_EVAL3(...) CG_PP_EVAL4(CG_PP_EVAL4(CG_PP_EVAL4(__VA_ARGS__)))
#define CG_PP_EVAL4(...) __VA_ARGS__

#define CG_PP_FOR_EACH_I(...) CG_PP_VAOPT_FOR_EACH_I(CG_PP_EXPAND_FRONT_APPLY, __VA_ARGS__)

#define CG_PP_HANDLE_COMMON(HandleType)               \
  HandleType(HandleType const &) noexcept = default;  \
  HandleType(HandleType &&) noexcept = default;       \
  HandleType &operator=(HandleType const &) = delete; \
  HandleType &operator=(HandleType &&) = delete

namespace compute_graph {

using size_t = std::size_t;

template <typename T> using SmallVector = CG_SMALL_VECTOR(T);

class NodeDescriptor;    // Describe how to create a node.
class SocketDescriptor;  // Describe a socket on a node.

class NodeBase;      // Base type for each node.
class InputSocket;   // a socket on a node.
class OutputSocket;  // a socket on a node.
class Graph;         // the context of the graph.
class Context;       // the context of node runtime.

using TypeIndex = std::type_index;

CG_STRONG_INLINE std::string to_string(TypeIndex type_index) { return type_index.name(); }

class SocketDescriptor {
public:
  CG_STRONG_INLINE SocketDescriptor(TypeIndex type, std::string_view name,
                                    std::string_view desc) noexcept
      : type_(type), name_(name), desc_(desc) {}

  CG_STRONG_INLINE SocketDescriptor(TypeIndex type, std::string_view name, std::string_view desc,
                                    std::string_view pretty_typename) noexcept
      : type_(type), name_(name), desc_(desc), pretty_typename_(pretty_typename) {}
  CG_STRONG_INLINE SocketDescriptor(SocketDescriptor const &) noexcept = default;
  CG_STRONG_INLINE SocketDescriptor(SocketDescriptor &&) noexcept = default;

  CG_STRONG_INLINE TypeIndex const &Type() const noexcept { return type_; }
  CG_STRONG_INLINE std::string const &Name() const noexcept { return name_; }
  CG_STRONG_INLINE std::string const &Desc() const noexcept { return desc_; }
  CG_STRONG_INLINE std::string const &PrettyTypename() const noexcept { return pretty_typename_; }

private:
  const TypeIndex type_;
  const std::string name_;
  const std::string desc_;
  const std::string pretty_typename_;
};

template <typename T>
CG_STRONG_INLINE SocketDescriptor make_socket_descriptor(std::string_view name,
                                                         std::string_view desc) {
  return {typeid(T), name, desc};
}

template <typename T>
CG_STRONG_INLINE SocketDescriptor make_socket_descriptor(std::string_view name,
                                                         std::string_view desc,
                                                         std::string_view pretty_typename) {
  return {typeid(T), name, desc, pretty_typename};
}

using NodeFactory = std::function<std::unique_ptr<NodeBase>(NodeDescriptor const &)>;

class NodeDescriptor {
public:
  CG_STRONG_INLINE SmallVector<SocketDescriptor> const &GetInputs() const noexcept {
    return inputs_;
  }

  CG_STRONG_INLINE SmallVector<SocketDescriptor> const &GetOutputs() const noexcept {
    return outputs_;
  }

  CG_STRONG_INLINE std::string const &Name() const noexcept { return name_; }

  CG_STRONG_INLINE std::string const &Desc() const noexcept { return desc_; }

  CG_STRONG_INLINE std::optional<size_t> FindInput(std::string_view name) const noexcept {
    for (size_t i = 0; i < inputs_.size(); ++i) {
      if (inputs_[i].Name() == name) {
        return i;
      }
    }
    return std::nullopt;
  }

  CG_STRONG_INLINE std::optional<size_t> FindOutput(std::string_view name) const noexcept {
    for (size_t i = 0; i < outputs_.size(); ++i) {
      if (outputs_[i].Name() == name) {
        return i;
      }
    }
    return std::nullopt;
  }

  CG_STRONG_INLINE NodeDescriptor(NodeDescriptor const &) = default;
  CG_STRONG_INLINE NodeDescriptor(NodeDescriptor &&) = default;

  CG_STRONG_INLINE std::unique_ptr<NodeBase> Build() const { return factory_(*this); }
  template <typename NodeType> friend class NodeDescriptorBuilder;

private:
  CG_STRONG_INLINE NodeDescriptor(std::string_view name, std::string_view desc,
                                  NodeFactory factory) noexcept
      : name_(name), desc_(desc), factory_(std::move(factory)) {}

  const std::string name_;
  const std::string desc_;
  NodeFactory factory_;
  SmallVector<SocketDescriptor> inputs_;
  SmallVector<SocketDescriptor> outputs_;
};

template <typename NodeType> class NodeDescriptorBuilder {
  static_assert(std::is_base_of_v<NodeBase, NodeType>, "NodeType must be derived from NodeBase");

public:
  CG_STRONG_INLINE NodeDescriptorBuilder(std::string name, std::string desc) noexcept
      : descriptor_(std::move(name), std::move(desc), [](NodeDescriptor const &descriptor) {
          return std::make_unique<NodeType>(descriptor);
        }) {}

  CG_STRONG_INLINE NodeDescriptorBuilder &AppendInput(SocketDescriptor const &desc) noexcept {
    descriptor_.inputs_.push_back(desc);
    return *this;
  }

  CG_STRONG_INLINE NodeDescriptorBuilder &AppendOutput(SocketDescriptor const &desc) noexcept {
    descriptor_.outputs_.push_back(desc);
    return *this;
  }

  CG_STRONG_INLINE NodeDescriptor Build() const noexcept { return descriptor_; }

private:
  NodeDescriptor descriptor_;
};

class NodeRegistry {
public:
  using container_type = std::unordered_map<std::string, NodeDescriptor>;
  using defered_load_fn = std::function<void(NodeRegistry &)>;

  CG_STRONG_INLINE auto Find(const std::string &type) const noexcept {
    return descriptors_.find(type);
  }

  const container_type &GetDescriptors() const noexcept { return descriptors_; }

  CG_STRONG_INLINE NodeDescriptor const &Emplace(NodeDescriptor const &descriptor) {
    return descriptors_.emplace(descriptor.Name(), descriptor).first->second;
  }

  CG_STRONG_INLINE void Emplace(NodeDescriptor &&descriptor) {
    descriptors_.emplace(descriptor.Name(), std::move(descriptor));
  }

  std::unique_ptr<NodeBase> Create(const std::string &type) const CG_NOEXCEPT {
    auto const it = descriptors_.find(type);
#ifndef CG_NO_CHECK
    if (it == descriptors_.end()) {
      CG_THROW(std::invalid_argument, "Invalid node type: " + type);
    }
#endif
    return it->second.Build();
  }

  template <typename T> std::unique_ptr<T> Create() const CG_NOEXCEPT {
    return std::unique_ptr<T>(static_cast<T *>(Create(T::name()).release()));
  }

  CG_STRONG_INLINE bool Has(std::string type) const noexcept {
    return descriptors_.find(type) != descriptors_.end();
  }

  void DeferedLoad(defered_load_fn loader) { defered_loader_.push_back(std::move(loader)); }

  void LoadDefered() {
    for (const auto &loader : defered_loader_) {
      loader(*this);
    }
    defered_loader_.clear();
  }

  bool IsDeferedLoaded() const noexcept { return defered_loader_.empty(); }

  CG_STRONG_INLINE void Clear() noexcept {
    defered_loader_.clear();
    descriptors_.clear();
  }

  static NodeRegistry &instance() {
    static NodeRegistry registry;
    return registry;
  }

  CG_STRONG_INLINE NodeRegistry() = default;
  NodeRegistry(NodeRegistry const &) = delete;
  NodeRegistry(NodeRegistry &&) = default;
  NodeRegistry &operator=(NodeRegistry const &) = delete;
  NodeRegistry &operator=(NodeRegistry &&) = default;

private:
  container_type descriptors_;
  std::vector<defered_load_fn> defered_loader_;
};

namespace intern {

template <size_t N, size_t Low, size_t High> struct StaticFor {
  template <template <size_t> typename Fn, typename... Args>
  static CG_STRONG_INLINE void eval(Args &&...args) {
    Fn<N>::eval(args...);
    StaticFor<N + 1, Low, High>().template eval<Fn>(args...);
  }
};

template <size_t N, size_t Low> struct StaticFor<N, Low, N> {
  template <template <size_t> typename Fn, typename... Args>
  static CG_STRONG_INLINE void eval(Args &&...) {}
};

template <size_t Low, size_t High, template <size_t> typename Fn, typename... Args>
CG_STRONG_INLINE void static_for_eval(Args &&...args) {
  StaticFor<Low, Low, High>().template eval<Fn, Args...>(std::forward<Args>(args)...);
}

template <typename T> struct IsMetaValid {
  static constexpr bool value = !std::is_same_v<typename T::type, void>;
};

template <template <size_t, typename> typename T, size_t current,
          bool valid = IsMetaValid<T<current, int>>::value>
struct count_meta;

template <template <size_t, typename> typename T, size_t current>
struct count_meta<T, current, true> {
  static constexpr size_t count = 1 + count_meta<T, current + 1>::count;
};
template <template <size_t, typename> typename T, size_t current>
struct count_meta<T, current, false> {
  static constexpr size_t count = 0;
};

template <template <size_t, typename> typename T> constexpr size_t count_meta_v
    = count_meta<T, 0>::count;

template <typename T> constexpr size_t count_socket_v = count_meta_v<T::template socket_meta>;

template <typename, typename T> struct HasDefaultValue {
  static_assert(std::integral_constant<T, false>::value,
                "Second template parameter needs to be of function type.");
};

template <typename C, typename Ret> struct HasDefaultValue<C, Ret()> {
private:
  template <typename T> static constexpr auto check(T *) ->
      typename std::is_same<decltype(T::DefaultValue()), Ret>::type;

  template <typename> static constexpr std::false_type check(...);

  using type = decltype(check<C>(nullptr));

public:
  static constexpr bool value = type::value;
};

template <typename C> struct HasOnRegister {
private:
  template <typename T> static constexpr auto check(T *) ->
      typename std::is_same<decltype(T::OnRegister()), void>::type;

  template <typename> static constexpr std::false_type check(...);

  using type = decltype(check<C>(nullptr));

public:
  static constexpr bool value = type::value;
};

template <typename C> struct HasOnConstruct {
private:
  template <typename T> static constexpr auto check(T *) ->
      typename std::is_same<decltype(std::declval<T>().OnConstruct()), void>::type;

  template <typename> static constexpr std::false_type check(...);

  using type = decltype(check<C>(nullptr));

public:
  static constexpr bool value = type::value;
};

template <typename C, bool is_valid_call = HasOnRegister<C>::value>
struct call_on_register_if_presented;

template <typename C> struct call_on_register_if_presented<C, true> {
  static CG_STRONG_INLINE void exec() { C::OnRegister(); }
};

template <typename C> struct call_on_register_if_presented<C, false> {
  static CG_STRONG_INLINE void exec() {}
};

template <typename C, bool is_valid_call = HasOnConstruct<C>::value>
struct call_on_construct_if_presented;

template <typename C> struct call_on_construct_if_presented<C, true> {
  static CG_STRONG_INLINE void exec(C &c) { c.OnConstruct(); }
};

template <typename C> struct call_on_construct_if_presented<C, false> {
  static CG_STRONG_INLINE void exec(C &) {}
};

template <typename M> static constexpr bool HasDefaultValue_v
    = HasDefaultValue<M, typename M::type const &()>::value;

template <typename M> static constexpr bool HasOnRegister_v = HasOnRegister<M>::value;

template <typename M> static constexpr bool HasOnConstruct_v = HasOnConstruct<M>::value;

struct SocketMetaBase {};

template <typename T> using is_socket_meta = std::is_base_of<SocketMetaBase, T>;

template <typename T> constexpr bool is_socket_meta_v = is_socket_meta<T>::value;

template <typename C, typename MT> struct HasOnConnectDispatch {
private:
  template <typename T> static constexpr auto check(T *) ->
      typename std::is_same<decltype(std::declval<T>().OnConnectDispatch(std::declval<MT>())),
                            void>::type;

  template <typename> static constexpr std::false_type check(...);

  using type = decltype(check<C>(nullptr));

public:
  static constexpr bool value = type::value;
};

template <typename C, typename MT> static constexpr bool HasOnConnectMt_v
    = HasOnConnectDispatch<C, MT>::value;

template <typename C, typename MT, bool = HasOnConnectMt_v<C, MT>>
struct call_on_connect_mt_if_presented;

template <typename C, typename MT> struct call_on_connect_mt_if_presented<C, MT, true> {
  static CG_STRONG_INLINE void exec(C &c, MT m) { c.OnConnectDispatch(m); }
};

template <typename C, typename MT> struct call_on_connect_mt_if_presented<C, MT, false> {
  static CG_STRONG_INLINE void exec(C &, MT) {}
};

}  // namespace intern

template <typename MT> struct IsSocketMeta : intern::is_socket_meta<MT> {};
template <typename MT> constexpr bool is_socket_meta_v = intern::is_socket_meta_v<MT>;
template <typename MT> struct HasDefaultValue : intern::HasDefaultValue<MT, typename MT::type> {};
template <typename MT> constexpr bool HasDefaultValue_v = intern::HasDefaultValue_v<MT>;
template <typename MT> struct HasOnRegister : intern::HasOnRegister<MT> {};
template <typename MT> constexpr bool HasOnRegister_v = intern::HasOnRegister_v<MT>;
template <typename MT> struct HasOnConstruct : intern::HasOnConstruct<MT> {};
template <typename MT> constexpr bool HasOnConstruct_v = intern::HasOnConstruct_v<MT>;
template <typename C, typename MT> struct HasOnConnectDispatch
    : intern::HasOnConnectDispatch<C, MT> {};
template <typename C, typename MT> constexpr bool HasOnConnectMt_v
    = intern::HasOnConnectMt_v<C, MT>;

using TypeErasedPtr = std::shared_ptr<void>;
using WeakTypeErasedPtr = std::weak_ptr<void>;

class SocketBase {
public:
  virtual bool IsInput() const noexcept = 0;
  virtual ~SocketBase() noexcept = default;
};

class OutputSocket : public SocketBase {
public:
  template <typename T, typename... Args> T &Emplace(Args &&...args) CG_NOEXCEPT {
#ifndef CG_NO_CHECK
    if (typeid(T) != type_) {
      CG_THROW(std::invalid_argument, "Type mismatch");
    }
#endif
    payload_ = std::make_shared<T>(std::forward<Args>(args)...);
    dirty_ = true;
    return *static_cast<T *>(payload_.get());
  }

  bool IsInput() const noexcept override { return false; }

  CG_STRONG_INLINE TypeIndex const &Type() const noexcept { return type_; }
  CG_STRONG_INLINE auto const &ConnectedSockets() const noexcept { return connected_sockets_; }

  CG_STRONG_INLINE TypeErasedPtr const &Payload() const noexcept { return payload_; }
  CG_STRONG_INLINE size_t Index() const noexcept { return index_; }
  CG_STRONG_INLINE NodeBase &Node() const noexcept { return node_; }

  CG_STRONG_INLINE void Clear() noexcept {
    payload_.reset();
    dirty_ = true;
  }
  CG_STRONG_INLINE bool IsDirty() const noexcept { return dirty_; }
  CG_STRONG_INLINE void MarkClean() noexcept { dirty_ = false; }

  CG_STRONG_INLINE OutputSocket(OutputSocket const &) = delete;
  CG_STRONG_INLINE OutputSocket(OutputSocket &&) noexcept = default;
  CG_STRONG_INLINE OutputSocket &operator=(OutputSocket const &) = delete;
  CG_STRONG_INLINE OutputSocket &operator=(OutputSocket &&) = delete;
  ~OutputSocket() noexcept override = default;

private:
  CG_STRONG_INLINE OutputSocket(TypeIndex type, NodeBase &node, size_t index) noexcept
      : type_(type), payload_{nullptr}, dirty_(false), node_(node), index_(index) {}

  CG_STRONG_INLINE void Erase(InputSocket const &to) noexcept {
    connected_sockets_.erase(std::remove_if(connected_sockets_.begin(), connected_sockets_.end(),
                                            [&to](auto const &socket) { return socket == &to; }),
                             connected_sockets_.end());
  }

  CG_STRONG_INLINE TypeErasedPtr &Payload() noexcept { return payload_; }
  CG_STRONG_INLINE void Connect(InputSocket const &to) noexcept {
    connected_sockets_.push_back(&to);
  }

  friend class NodeBase;
  friend class Graph;
  TypeIndex const type_;
  SmallVector<InputSocket const *> connected_sockets_;
  TypeErasedPtr payload_;
  bool dirty_;
  NodeBase &node_;
  size_t const index_;
};

class InputSocket : public SocketBase {
public:
  bool IsInput() const noexcept override { return true; }
  CG_STRONG_INLINE TypeIndex const &Type() const noexcept { return type_; }
  CG_STRONG_INLINE TypeErasedPtr FetchPayload() const noexcept {
    if (from_ == nullptr) {
      return nullptr;
    }

    if (fetched_payload_.expired()) {
      fetched_payload_ = from_->Payload();
    }
    return fetched_payload_.lock();
  }
  CG_STRONG_INLINE bool IsConnected() const noexcept { return from_ != nullptr; }
  CG_STRONG_INLINE bool IsEmpty() const noexcept { return FetchPayload() == nullptr; }

  CG_STRONG_INLINE OutputSocket const *From() const noexcept { return from_; }
  CG_STRONG_INLINE NodeBase &Node() const noexcept { return node_; }
  CG_STRONG_INLINE size_t Index() const noexcept { return index_; }

  CG_STRONG_INLINE InputSocket(InputSocket const &) = delete;
  CG_STRONG_INLINE InputSocket(InputSocket &&) noexcept = default;
  CG_STRONG_INLINE InputSocket &operator=(InputSocket const &) = delete;
  CG_STRONG_INLINE InputSocket &operator=(InputSocket &&) = delete;
  ~InputSocket() noexcept override = default;

private:
  CG_STRONG_INLINE InputSocket(TypeIndex type, NodeBase &node, size_t const index) noexcept
      : type_(type), node_(node), index_(index), from_{nullptr} {}
  CG_STRONG_INLINE void Clear() noexcept {
    from_ = nullptr;
    fetched_payload_.reset();
  }
  CG_STRONG_INLINE void Connect(OutputSocket const *from) noexcept { from_ = from; }

  friend class NodeBase;
  friend class Graph;
  const TypeIndex type_;
  NodeBase &node_;
  size_t const index_;
  OutputSocket const *from_;
  mutable WeakTypeErasedPtr fetched_payload_;
};

class NodeBase CG_NODE_INHERITANCE {
public:
  CG_STRONG_INLINE explicit NodeBase(NodeDescriptor const &descriptor,
                                     bool do_prepare = true) noexcept
      : descriptor_(descriptor) {
    if (do_prepare) {
      prepare_sockets(descriptor);
    }
  }

  NodeBase(NodeBase const &) = delete;
  NodeBase(NodeBase &&) noexcept = default;
  NodeBase &operator=(NodeBase const &) = delete;
  NodeBase &operator=(NodeBase &&) = delete;

  virtual ~NodeBase() noexcept = default;

  // execute, may throw exception.
  virtual void operator()(Context &) = 0;

  CG_STRONG_INLINE NodeDescriptor const &GetDescriptor() const noexcept { return descriptor_; }

  CG_STRONG_INLINE auto const &GetInputs() const noexcept { return inputs_; }
  CG_STRONG_INLINE auto const &GetOutputs() const noexcept { return outputs_; }
  CG_STRONG_INLINE auto const &GetPrecedents() const noexcept { return prec_; }
  CG_STRONG_INLINE auto const &GetSuccessors() const noexcept { return succ_; }

  std::optional<size_t> FindInput(std::string_view name) const noexcept {
    return descriptor_.FindInput(name);
  }

  std::optional<size_t> FindOutput(std::string_view name) const noexcept {
    return descriptor_.FindOutput(name);
  }

  CG_NODE_EXTENSION

protected:
  virtual void OnConnect(size_t /*index*/) noexcept {}

  CG_STRONG_INLINE TypeErasedPtr Get(size_t index) const {
#ifndef CG_NO_CHECK
    return inputs_.at(index).FetchPayload();
#else
    return inputs_[index].FetchPayload();
#endif
  }

  template <typename T> CG_STRONG_INLINE T const *Get(size_t index) const CG_NOEXCEPT {
#ifndef CG_NO_CHECK
    if (inputs_[index].Type() != typeid(T)) {
      CG_THROW(std::invalid_argument, "Type mismatch");
    }
#endif
    return static_cast<T const *>(Get(index).get());
  }

  CG_STRONG_INLINE bool Has(size_t index) const noexcept {
#ifndef CG_NO_CHECK
    return !inputs_.at(index).IsEmpty();
#else
    return !inputs_[index].IsEmpty();
#endif
  }

  template <typename T, typename... Args> CG_STRONG_INLINE auto &Set(size_t index, Args &&...args) {
    return outputs_[index].Emplace<T>(std::forward<Args>(args)...);
  }

  CG_STRONG_INLINE void prepare_sockets(NodeDescriptor const &descriptor) {
    inputs_.reserve(descriptor.GetInputs().size());
    outputs_.reserve(descriptor.GetOutputs().size());

    for (size_t i = 0; i < descriptor.GetInputs().size(); ++i) {
      inputs_.push_back({descriptor.GetInputs()[i].Type(), *this, i});
    }

    for (size_t i = 0; i < descriptor.GetOutputs().size(); ++i) {
      outputs_.push_back({descriptor.GetOutputs()[i].Type(), *this, i});
    }
  }

  SmallVector<InputSocket> inputs_;    ///< Input sockets.
  SmallVector<OutputSocket> outputs_;  ///< Output sockets.
  SmallVector<NodeBase *> prec_;       ///< Nodes that this node depends on.
  SmallVector<NodeBase *> succ_;       ///< Nodes that depend on this node.

private:
  friend class Graph;
  NodeDescriptor const &descriptor_;  ///< Node descriptor
};

// crtp.
template <typename Derived> class NodeDerive : public NodeBase {
public:
  CG_STRONG_INLINE explicit NodeDerive(NodeDescriptor const &descriptor) noexcept
      : NodeBase(descriptor) {}

  ~NodeDerive() noexcept override = default;

  struct InternNodeTraits {
    using input_metas = typename Derived::intern_input_meta;
    using output_metas = typename Derived::intern_output_meta;
    template <size_t i> struct InputRegFn {
      static CG_STRONG_INLINE void eval(NodeDescriptorBuilder<Derived> &builder) noexcept {
        using meta = typename input_metas::template socket_meta<i>;
        using T = typename meta::type;
        builder.AppendInput(
            make_socket_descriptor<T>(meta::name, meta::description, meta::pretty_typename));
      }
    };

    template <size_t i> struct OutputRegFn {
      static CG_STRONG_INLINE void eval(NodeDescriptorBuilder<Derived> &builder) noexcept {
        using meta = typename output_metas::template socket_meta<i>;
        using T = typename meta::type;
        builder.AppendOutput(
            make_socket_descriptor<T>(meta::name, meta::description, meta::pretty_typename));
      }
    };

    template <size_t i> struct InputOnConnectFn {
      static CG_STRONG_INLINE void eval(size_t index, Derived &node) noexcept {
        using meta = typename input_metas::template socket_meta<i>;
        if (index == i) {
          intern::call_on_connect_mt_if_presented<Derived, meta>::exec(node, meta{});
        }
      }
    };

    template <size_t i> struct InputDefaultValueFn {
      template <bool, typename> struct AvoidIfConstexpr {
        CG_STRONG_INLINE static void eval(size_t, void const *&) {}
      };

      template <typename Dummy> struct AvoidIfConstexpr<true, Dummy> {
        CG_STRONG_INLINE static void eval(size_t index, void const *&data) {
          using meta = typename input_metas::template socket_meta<i>;
          if (index == i) {
            data = &meta::DefaultValue();
          }
        }
      };

      static CG_STRONG_INLINE void eval(size_t index, void const *&data) noexcept {
        using meta = typename input_metas::template socket_meta<i>;
        AvoidIfConstexpr<HasDefaultValue_v<meta>, void>::eval(index, data);
      }
    };
    static constexpr size_t num_inputs = intern::count_socket_v<input_metas>;
    static constexpr size_t num_outputs = intern::count_socket_v<output_metas>;
  };

  static CG_STRONG_INLINE NodeDescriptor BuildDescriptor() {
    NodeDescriptorBuilder<Derived> builder(Derived::name(), Derived::desc());
    intern::static_for_eval<0, InternNodeTraits::num_inputs, InternNodeTraits::template InputRegFn>(
        builder);
    intern::static_for_eval<0, InternNodeTraits::num_outputs,
                            InternNodeTraits::template OutputRegFn>(builder);
    return builder.Build();
  }

  static CG_STRONG_INLINE void RegisterTo(NodeRegistry &registry) {
    registry.DeferedLoad([](NodeRegistry &registry) {
      auto const &descriptor = BuildDescriptor();
      intern::call_on_register_if_presented<Derived>::exec();
      registry.Emplace(descriptor);
    });
  }

protected:
  CG_STRONG_INLINE void OnConnect(size_t index) noexcept override {
    constexpr size_t total = InternNodeTraits::num_inputs;
    intern::static_for_eval<0, total, InternNodeTraits::template InputOnConnectFn>(
        index, *static_cast<Derived *>(this));
  }

  using NodeBase::Set, NodeBase::Has;

  template <typename MT, typename = std::enable_if_t<intern::is_socket_meta_v<MT>>>
  CG_STRONG_INLINE auto Get(MT) const noexcept -> std::add_const_t<typename MT::type> * {
    constexpr size_t index = MT::index;
    return static_cast<typename MT::type const *>(NodeBase::Get(index).get());
  }

  template <typename MT, typename = std::enable_if_t<intern::is_socket_meta_v<MT>>>
  CG_STRONG_INLINE auto Ensure(MT) const CG_NOEXCEPT->std::add_const_t<typename MT::type> & {
    if (typename MT::type const *ptr = Get<MT>({})) {
      return *ptr;
    }
    CG_THROW(std::runtime_error, "Socket " + std::string(MT::name) +" not connected.");
  }

  template <typename T, typename = std::enable_if_t<!intern::is_socket_meta_v<T>>>
  CG_STRONG_INLINE T const *Get(size_t index) const {
    return NodeBase::Get<T>(index);
  }

  template <typename MT, typename = std::enable_if_t<intern::is_socket_meta_v<MT>>>
  CG_STRONG_INLINE auto const &GetOr(MT) const {
    constexpr size_t index = MT::index;
    return Has(index) ? *Get<MT>({}) : DefaultValue<MT>({});
  }

  template <typename T, typename = std::enable_if_t<!intern::is_socket_meta_v<T>>>
  CG_STRONG_INLINE T const *GetOr(size_t index) const {
    return static_cast<T const *>(Has(index) ? Get<T>(index) : DefaultValue(index));
  }

  template <typename MT, typename... Args,
            typename = std::enable_if_t<intern::is_socket_meta_v<MT>>>
  CG_STRONG_INLINE auto &Set(MT, Args &&...args) {
    constexpr size_t index = MT::index;
    using T = typename MT::type;
    return Set<T>(index, std::forward<Args>(args)...);
  }

  template <typename MT, typename = std::enable_if_t<intern::is_socket_meta_v<MT>>>
  static CG_STRONG_INLINE auto const &DefaultValue(MT) noexcept {
    static_assert(intern::HasDefaultValue_v<MT>, "No default value.");
    return MT::DefaultValue();
  }

  template <typename T, typename = std::enable_if_t<std::is_convertible_v<T, size_t>>>
  static CG_STRONG_INLINE void const *DefaultValue(T index) CG_NOEXCEPT {
#ifndef CG_NO_CHECK
    if (index >= InternNodeTraits::num_inputs) {
      CG_THROW(std::out_of_range, "Input index out of range.");
    }
#endif
    void const *data = nullptr;
    auto const ind = static_cast<size_t>(index);
    constexpr size_t total = InternNodeTraits::num_inputs;
    intern::static_for_eval<0, total, InternNodeTraits::template InputDefaultValueFn>(ind, data);
    return data;
  }

  template <typename MT, typename = std::enable_if_t<intern::is_socket_meta_v<MT>>>
  CG_STRONG_INLINE bool Has(MT) const noexcept {
    constexpr size_t index = MT::index;
    return Has(index);
  }

  template <size_t i> static CG_STRONG_INLINE auto const &DefaultValue() noexcept {
    static_assert(i < InternNodeTraits::num_inputs, "Index out of range.");
    using MT = typename InternNodeTraits::input_metas::template socket_meta<i>;
    return DefaultValue<MT>({});
  }

private:
  template <size_t... Idx> auto GetAllImpl(std::index_sequence<Idx...>) const {
    using inputs = typename InternNodeTraits::input_metas;
    return std::make_tuple(Get<typename inputs::template socket_meta<Idx, int>>({})...);
  }

  template <typename... Args, size_t... Idx>
  auto SetAllImpl(std::index_sequence<Idx...>, std::tuple<Args...> arg) {
    using outputs = typename InternNodeTraits::output_metas;
    return std::make_tuple(
        Set<typename outputs::template socket_meta<Idx, int>::type>(Idx, std::get<Idx>(arg))...);
  }

protected:
  CG_STRONG_INLINE auto GetAll() const {
    return GetAllImpl(std::make_index_sequence<InternNodeTraits::num_inputs>());
  }

  template <typename... Args> CG_STRONG_INLINE auto SetAll(Args &&...args) {
    return SetAllImpl(std::make_index_sequence<InternNodeTraits::num_outputs>(),
                      std::tuple<Args &&...>(std::forward<Args>(args)...));
  }

private:
  using NodeBase::inputs_, NodeBase::outputs_;
};

using ctx_frame = std::unordered_map<std::string, std::any>;
using stacked_frames = std::deque<ctx_frame>;

class Context {
public:
  Context() noexcept;
  Context(const Context &) = delete;
  Context &operator=(const Context &) = delete;
  Context(Context &&) noexcept = default;
  Context &operator=(Context &&) = delete;

  // Stack related.
  void PushStack() noexcept;
  void PushStack(ctx_frame frame) noexcept;
  void PopStack() noexcept;
  size_t StackSize() const noexcept;
  ctx_frame const &Top() const noexcept;
  ctx_frame &Top() noexcept;
  stacked_frames &Frames() noexcept;
  stacked_frames const &Frames() const noexcept;

  // Frame related.
  // Put a value in the current frame.
  auto Emplace(std::string_view key, std::any const &value);
  template <typename T, typename... Args> auto Emplace(std::string_view key, Args &&...args);

  // Get a value from the current frame.
  std::any const &Get(std::string const &key) const CG_NOEXCEPT;
  std::any const &GetTop(std::string const &key) const CG_NOEXCEPT;

  void Clear() noexcept;

  bool Has(std::string const &key) const noexcept;
  bool HasTop(std::string const &key) const noexcept;

private:
  stacked_frames frames_;
};

CG_STRONG_INLINE Context::Context() noexcept = default;

CG_STRONG_INLINE void Context::PushStack() noexcept { frames_.emplace_back(); }

CG_STRONG_INLINE void Context::PushStack(ctx_frame frame) noexcept {
  frames_.emplace_back(std::move(frame));
}

CG_STRONG_INLINE void Context::PopStack() noexcept {
  if (!frames_.empty()) frames_.pop_back();
}

CG_STRONG_INLINE size_t Context::StackSize() const noexcept { return frames_.size(); }

CG_STRONG_INLINE ctx_frame const &Context::Top() const noexcept { return frames_.back(); }

CG_STRONG_INLINE ctx_frame &Context::Top() noexcept { return frames_.back(); }

CG_STRONG_INLINE stacked_frames &Context::Frames() noexcept { return frames_; }

CG_STRONG_INLINE stacked_frames const &Context::Frames() const noexcept { return frames_; }

CG_STRONG_INLINE auto Context::Emplace(std::string_view key, std::any const &value) {
  return Top().emplace(key, value);
}

CG_STRONG_INLINE std::any const &Context::Get(std::string const &key) const CG_NOEXCEPT {
  for (auto it = frames_.crbegin(); it != frames_.crend(); ++it) {
    if (auto const &it2 = it->find(key); it2 != it->end()) return it2->second;
  }
  CG_THROW(std::out_of_range, "Key not found in context: " + std::string(key));
  CG_UNREACHABLE();
}

inline std::any const &Context::GetTop(std::string const &key) const CG_NOEXCEPT {
  if (auto const &it = Top().find(key); it != Top().end()) return it->second;
  CG_THROW(std::out_of_range, "Key not found in context: " + std::string(key));
  CG_UNREACHABLE();
}

CG_STRONG_INLINE bool Context::Has(std::string const &key) const noexcept {
  for (auto it = frames_.crbegin(); it != frames_.crend(); ++it) {
    if (auto const &it2 = it->find(key); it2 != it->end()) return true;
  }
  return false;
}

CG_STRONG_INLINE bool Context::HasTop(std::string const &key) const noexcept {
  auto const &it = Top().find(key);
  return it != Top().end();
}

CG_STRONG_INLINE void Context::Clear() noexcept { frames_.clear(); }

template <typename T, typename... Args>
CG_STRONG_INLINE auto Context::Emplace(std::string_view key, Args &&...args) {
  return Top().emplace(key, std::make_any<T>(std::forward<Args>(args)...));
}

template <typename NodeType = NodeBase> class InputSocketHandle;
template <typename NodeType = NodeBase> class OutputSocketHandle;
template <typename NodeType = NodeBase> class NodeHandle;
template <typename From = NodeBase, typename To = NodeBase> class Link;

template <typename NodeType> class NodeHandle {
public:
  CG_PP_HANDLE_COMMON(NodeHandle);

  template <typename AnotherNodeType,
            typename = std::enable_if_t<std::is_base_of_v<AnotherNodeType, NodeType>>>
  CG_STRONG_INLINE NodeHandle<AnotherNodeType> Cast() const noexcept {
    return {static_cast<AnotherNodeType &>(node_), index_};
  }

  template <typename AnotherNodeType,
            typename = std::enable_if_t<std::is_base_of_v<NodeType, AnotherNodeType>>>
  CG_STRONG_INLINE NodeHandle(NodeHandle<AnotherNodeType> const &another) noexcept
      : NodeHandle(another.template Cast<NodeType>()) {}

  CG_STRONG_INLINE NodeType const &operator*() const noexcept { return node_; }
  CG_STRONG_INLINE NodeType &operator*() noexcept { return node_; }
  CG_STRONG_INLINE NodeType const *operator->() const noexcept { return &node_; }
  CG_STRONG_INLINE NodeType *operator->() noexcept { return &node_; }
  CG_STRONG_INLINE NodeType &Node() noexcept { return node_; }
  CG_STRONG_INLINE NodeType const &Node() const noexcept { return node_; }
  CG_STRONG_INLINE size_t Index() const noexcept { return index_; }

  std::optional<InputSocketHandle<NodeType>> GetInput(std::string_view name) noexcept;
  std::optional<OutputSocketHandle<NodeType>> GetOutput(std::string_view name) noexcept;

  InputSocketHandle<NodeType> GetInput(size_t index);
  OutputSocketHandle<NodeType> GetOutput(size_t index);

  template <typename MT, typename = std::enable_if_t<intern::is_socket_meta_v<MT>>>
  InputSocketHandle<NodeType> GetInput(MT);
  template <typename MT, typename = std::enable_if_t<intern::is_socket_meta_v<MT>>>
  OutputSocketHandle<NodeType> GetOutput(MT);

private:
  NodeHandle(NodeType &node, size_t index) : node_(node), index_(index) {}

  NodeType &node_;
  const size_t index_;
  template <typename T> friend class NodeHandle;
  friend class Graph;
};

template <typename NodeType> class InputSocketHandle {
public:
  CG_PP_HANDLE_COMMON(InputSocketHandle);
  template <typename AnotherNodeType,
            typename = std::enable_if_t<std::is_base_of_v<AnotherNodeType, NodeType>>>
  CG_STRONG_INLINE InputSocketHandle<AnotherNodeType> Cast() const noexcept {
    return {static_cast<AnotherNodeType &>(node_), index_};
  }

  template <typename AnotherNodeType,
            typename = std::enable_if_t<std::is_base_of_v<NodeType, AnotherNodeType>>>
  CG_STRONG_INLINE InputSocketHandle(NodeHandle<AnotherNodeType> const &another) noexcept
      : InputSocketHandle(another.template cast<NodeType>()) {}

  CG_STRONG_INLINE InputSocket const &operator*() const noexcept {
    return node_.GetInputs()[index_];
  }
  CG_STRONG_INLINE InputSocket const *operator->() const noexcept {
    return &node_.GetInputs()[index_];
  }

  CG_STRONG_INLINE NodeType &Node() noexcept { return node_; }
  CG_STRONG_INLINE NodeType const &Node() const noexcept { return node_; }
  CG_STRONG_INLINE size_t Index() const noexcept { return index_; }

private:
  CG_STRONG_INLINE InputSocketHandle(NodeType &node, size_t index) : node_(node), index_(index) {}

  friend class Graph;
  friend class NodeHandle<NodeType>;
  template <typename T> friend class InputSocketHandle;
  template <typename From, typename To> friend class Link;

  NodeType &node_;
  size_t const index_;
};

template <typename NodeType> class OutputSocketHandle {
public:
  CG_PP_HANDLE_COMMON(OutputSocketHandle);
  template <typename AnotherNodeType,
            typename = std::enable_if_t<std::is_base_of_v<AnotherNodeType, NodeType>>>
  CG_STRONG_INLINE OutputSocketHandle<AnotherNodeType> Cast() const noexcept {
    return {static_cast<AnotherNodeType &>(node_), index_};
  }

  template <typename AnotherNodeType,
            typename = std::enable_if_t<std::is_base_of_v<NodeType, AnotherNodeType>>>
  CG_STRONG_INLINE OutputSocketHandle(NodeHandle<AnotherNodeType> const &another) noexcept
      : OutputSocketHandle(another.template cast<NodeType>()) {}

  CG_STRONG_INLINE OutputSocket const &operator*() const noexcept {
    return node_.GetOutputs()[index_];
  }
  CG_STRONG_INLINE OutputSocket const *operator->() const noexcept {
    return &node_.GetOutputs()[index_];
  }

  CG_STRONG_INLINE NodeType &Node() noexcept { return node_; }
  CG_STRONG_INLINE NodeType const &Node() const noexcept { return node_; }
  CG_STRONG_INLINE size_t Index() const noexcept { return index_; }

private:
  CG_STRONG_INLINE OutputSocketHandle(NodeType &node, size_t index) : node_(node), index_(index) {}

  friend class Graph;
  friend class NodeHandle<NodeType>;
  template <typename T> friend class OutputSocketHandle;
  template <typename From, typename To> friend class Link;
  NodeType &node_;
  size_t const index_;
};

template <typename NodeFrom, typename NodeTo> class Link {
public:
  using FromType = OutputSocketHandle<NodeFrom>;
  using ToType = InputSocketHandle<NodeTo>;

  template <typename AnotherFrom, typename AnotherTo>
  CG_STRONG_INLINE Link<AnotherFrom, AnotherTo> Cast() const noexcept {
    return {from_.template cast<AnotherFrom>(), to_.template cast<AnotherTo>()};
  }

  template <typename AnotherFrom, typename AnotherTo>
  CG_STRONG_INLINE Link(Link<AnotherFrom, AnotherTo> const &another) noexcept
      : Link(another.template Cast<AnotherFrom, AnotherTo>()) {}

  CG_PP_HANDLE_COMMON(Link);
  CG_STRONG_INLINE FromType const &From() const noexcept { return from_; }
  CG_STRONG_INLINE ToType const &To() const noexcept { return to_; }

private:
  CG_STRONG_INLINE Link(FromType from, ToType to) noexcept : from_(from), to_(to) {}

  friend class Graph;
  FromType from_;
  ToType to_;
};

template <typename NodeFrom, typename NodeTo> class StrongLink {
public:
  using FromType = NodeHandle<NodeFrom>;
  using ToType = NodeHandle<NodeTo>;

  template <typename AnotherFrom, typename AnotherTo>
  CG_STRONG_INLINE StrongLink<AnotherFrom, AnotherTo> Cast() const noexcept {
    return {from_.template Cast<AnotherFrom>(), to_.template Cast<AnotherTo>()};
  }

  template <typename AnotherFrom, typename AnotherTo>
  CG_STRONG_INLINE StrongLink(StrongLink<AnotherFrom, AnotherTo> const &another) noexcept
      : StrongLink(another.template Cast<AnotherFrom, AnotherTo>()) {}

  StrongLink(StrongLink const &) noexcept = default;
  StrongLink(StrongLink &&) noexcept = default;
  StrongLink &operator=(StrongLink const &) = delete;
  StrongLink &operator=(StrongLink &&) = delete;
  CG_STRONG_INLINE FromType const &From() const noexcept { return from_; }
  CG_STRONG_INLINE ToType const &To() const noexcept { return to_; }

private:
  CG_STRONG_INLINE StrongLink(FromType from, ToType to) noexcept : from_(from), to_(to) {}
  friend class Graph;
  FromType from_;
  ToType to_;
};

class Graph {
public:
  using node_ptr = std::unique_ptr<NodeBase>;
  using node_container = std::vector<node_ptr>;
  using id_container = std::vector<size_t>;
  using node_addr_to_index_map = std::map<NodeBase const *, size_t>;

  Graph() = default;
  Graph(Graph const &) = delete;
  Graph(Graph &&) = default;
  ~Graph() noexcept { Clear(); }
  void Clear() noexcept;
  CG_STRONG_INLINE size_t NumNodes() const noexcept { return nodes_.size() - free_ids_.size(); }
  CG_STRONG_INLINE size_t NumLinks() const noexcept { return link_size_; }

  CG_STRONG_INLINE node_container const &GetNodes() const noexcept { return nodes_; }
  CG_STRONG_INLINE node_container &GetNodes() noexcept { return nodes_; }

  NodeHandle<NodeBase> operator[](size_t index) noexcept { return {*nodes_[index], index}; }

  // node op.
  template <typename NodeType>
  NodeHandle<NodeType> PushBack(std::unique_ptr<NodeType> node) noexcept;
  template <typename NodeType>
  std::optional<NodeHandle<NodeType>> Get(NodeType const *ptr) const noexcept;
  template <typename NodeType> void Erase(NodeHandle<NodeType> handle) noexcept;

  // strong link, describe node-wise connection.
  template <typename From, typename To>
  StrongLink<From, To> Connect(NodeHandle<From> prec, NodeHandle<To> succ) noexcept;
  template <typename From, typename To>
  bool HasConnect(NodeHandle<From> prec, NodeHandle<To> succ) noexcept;
  template <typename From, typename To>
  std::optional<StrongLink<From, To>> GetConnect(NodeHandle<From> prec,
                                                 NodeHandle<To> succ) noexcept;
  template <typename From, typename To> void Erase(StrongLink<From, To> link) noexcept;

  // socket op
  template <typename From, typename To>
  Link<From, To> Connect(OutputSocketHandle<From> from, InputSocketHandle<To> to);
  template <typename From, typename To>
  bool HasConnect(OutputSocketHandle<From> const &from,
                  InputSocketHandle<To> const &to) const noexcept;
  template <typename From, typename To>
  std::optional<Link<From, To>> GetConnect(OutputSocketHandle<From> const &from,
                                           InputSocketHandle<To> const &to) const noexcept;
  template <typename From, typename To> void Erase(Link<From, To> link) noexcept;

  void TopologySort();
  std::vector<size_t> TopologyOrder() const noexcept;
  bool HasCycle() const noexcept;
  void ShrinkToFit() noexcept;

  const node_addr_to_index_map &AddrToIndex() const noexcept { return addr_to_index_; }

private:
  void RebuildAddrToIndex() noexcept;

  node_container nodes_;
  id_container free_ids_;
  node_addr_to_index_map addr_to_index_;
  size_t uid_next_ = 0;
  size_t link_size_ = 0;
  bool is_sorted_ = false;
};

CG_STRONG_INLINE bool can_connect(OutputSocket const &from, InputSocket const &to) noexcept {
  return from.Type() == to.Type();
}

template <typename NodeType>
CG_STRONG_INLINE std::optional<InputSocketHandle<NodeType>> NodeHandle<NodeType>::GetInput(
    std::string_view name) noexcept {
  if (auto const index = node_.FindInput(name)) {
    return GetInput(*index);
  }
  return std::nullopt;
}

template <typename NodeType>
CG_STRONG_INLINE std::optional<OutputSocketHandle<NodeType>> NodeHandle<NodeType>::GetOutput(
    std::string_view name) noexcept {
  if (const auto index = node_.FindOutput(name)) {
    return GetInput(*index);
  }
  return std::nullopt;
}

template <typename NodeType>
CG_STRONG_INLINE InputSocketHandle<NodeType> NodeHandle<NodeType>::GetInput(size_t index) {
  return {node_, index};
}

template <typename NodeType> template <typename MT, typename>
CG_STRONG_INLINE InputSocketHandle<NodeType> NodeHandle<NodeType>::GetInput(MT) {
  return GetInput(MT::index);
}

template <typename NodeType>
CG_STRONG_INLINE OutputSocketHandle<NodeType> NodeHandle<NodeType>::GetOutput(size_t index) {
  return {node_, index};
}

template <typename NodeType> template <typename MT, typename>
CG_STRONG_INLINE OutputSocketHandle<NodeType> NodeHandle<NodeType>::GetOutput(MT) {
  return GetOutput(MT::index);
}

CG_STRONG_INLINE void Graph::Clear() noexcept {
  while (!nodes_.empty()) {
    if (nodes_.back()) {
      Erase(NodeHandle{*nodes_.back(), nodes_.size() - 1});
    }
    nodes_.pop_back();
  }
  nodes_.clear();
  free_ids_.clear();
  addr_to_index_.clear();
  uid_next_ = 0;
  link_size_ = 0;
  is_sorted_ = false;
}

template <typename NodeType>
CG_STRONG_INLINE NodeHandle<NodeType> Graph::PushBack(std::unique_ptr<NodeType> node) noexcept {
  if (free_ids_.empty()) {
    free_ids_.push_back(nodes_.size());
    nodes_.push_back(nullptr);
  }
  size_t const index = free_ids_.back();
  free_ids_.pop_back();
  nodes_[index] = std::move(node);
  addr_to_index_.insert({nodes_[index].get(), index});
  is_sorted_ = false;
  return {*static_cast<NodeType *>(nodes_[index].get()), index};
}

template <typename NodeType> CG_STRONG_INLINE std::optional<NodeHandle<NodeType>> Graph::Get(
    NodeType const *ptr) const noexcept {
  if (auto it = addr_to_index_.find(static_cast<NodeBase const *>(ptr));
      it != addr_to_index_.end()) {
    return NodeHandle<NodeType>{*nodes_[it->second], it->second};
  }
  return std::nullopt;
}

template <typename NodeType>
CG_STRONG_INLINE void Graph::Erase(NodeHandle<NodeType> handle) noexcept {
  size_t const index = handle.Index();
  for (size_t i = 0; i < handle->GetInputs().size(); ++i) {
    if (auto const &input_sock = handle->GetInputs()[i]; input_sock.IsConnected()) {
      auto const &output_sock = *input_sock.From();
      Erase(Link{{output_sock.Node(), output_sock.Index()}, {handle.Node(), i}});
    }
  }

  for (size_t i = 0; i < handle->GetOutputs().size(); ++i) {
    auto const &output_sock = handle->GetOutputs()[i];
    while (!output_sock.ConnectedSockets().empty()) {
      const auto &input_sock = output_sock.ConnectedSockets().back();
      Erase(Link{{handle.Node(), i}, {input_sock->Node(), input_sock->Index()}});
    }
  }

  NodeBase &node = *handle;
  for (size_t i = 0; i < node.prec_.size(); ++i) {
    auto &prec = *node.prec_[i];
    if (auto it = std::find(prec.succ_.begin(), prec.succ_.end(), &node); it != prec.succ_.end()) {
      prec.succ_.erase(it);
    }
  }
  for (size_t i = 0; i < node.succ_.size(); ++i) {
    auto &succ = *node.succ_[i];
    if (auto it = std::find(succ.prec_.begin(), succ.prec_.end(), &node); it != succ.prec_.end()) {
      succ.prec_.erase(it);
    }
  }

  addr_to_index_.erase(nodes_[index].get());
  nodes_[index].reset();
  free_ids_.push_back(index);
  // is_sorted_ = false; // NOTE: Erase a node does not affect the topology order.
}

template <typename From, typename To>
CG_STRONG_INLINE StrongLink<From, To> Graph::Connect(NodeHandle<From> prec,
                                                     NodeHandle<To> succ) noexcept {
  NodeBase &p = prec.Node(), &s = succ.Node();
  if (!HasConnect(prec, succ)) {
    p.succ_.push_back(&s);
    s.prec_.push_back(&p);
  }
  is_sorted_ = false;
  return {prec, succ};
}

template <typename From, typename To>
bool Graph::HasConnect(NodeHandle<From> prec, NodeHandle<To> succ) noexcept {
  const NodeBase &n = prec.Node();
  const NodeBase &succ_node = succ.Node();
  return std::find(n.succ_.begin(), n.succ_.end(), &succ_node) != n.succ_.end();
}

template <typename From, typename To>
std::optional<StrongLink<From, To>> Graph::GetConnect(NodeHandle<From> prec,
                                                      NodeHandle<To> succ) noexcept {
  if (HasConnect(prec, succ)) {
    return StrongLink{prec, succ};
  }
  return std::nullopt;
}

template <typename From, typename To> void Graph::Erase(StrongLink<From, To> link) noexcept {
  NodeBase &p = link.From().Node();
  NodeBase &s = link.To().Node();
  if (auto pit = std::find(p.succ_.begin(), p.succ_.end(), &s); pit != p.succ_.end()) {
    p.succ_.erase(pit);
  }
  if (auto sit = std::find(s.prec_.begin(), s.prec_.end(), &p); sit != s.prec_.end()) {
    s.prec_.erase(sit);
  }
  is_sorted_ = false;
}

template <typename From, typename To>
CG_STRONG_INLINE Link<From, To> Graph::Connect(OutputSocketHandle<From> from,
                                               InputSocketHandle<To> to) CG_NOEXCEPT {
  if (!can_connect(*from, *to)) {
    CG_THROW(std::invalid_argument, "Cannot connect sockets of different types."
                                        + to_string(from->Type()) + " and "
                                        + to_string(to->Type()));
  }

  // If already connected, erase the old link.
  if (to->IsConnected()) {
    OutputSocket const *previous_from = to->From();
    Erase(Link{{previous_from->Node(), previous_from->Index()}, {to->Node(), to->Index()}});
  }

  // add link in between.
  NodeBase &from_node = from->Node(), &to_node = to->Node();
  from_node.outputs_[from.Index()].Connect(*to);
  to_node.inputs_[to.Index()].Connect(from.operator->());
  ++link_size_;
  is_sorted_ = false;

  to->Node().OnConnect(to.Index());
  return {from, to};
}

template <typename From, typename To>
CG_STRONG_INLINE bool Graph::HasConnect(OutputSocketHandle<From> const &from,
                                        InputSocketHandle<To> const &to) const noexcept {
  return to->From() == from.operator->();
}

template <typename From, typename To>
std::optional<Link<From, To>> Graph::GetConnect(OutputSocketHandle<From> const &from,
                                                InputSocketHandle<To> const &to) const noexcept {
  if (HasConnect(from, to)) {
    return Link{from, to};
  }
  return std::nullopt;
}

template <typename From, typename To>
CG_STRONG_INLINE void Graph::Erase(Link<From, To> link) noexcept {
  auto &from = link.from_.node_;
  auto &to = link.to_.node_;
  auto &from_sock = from.outputs_[link.From().Index()];
  auto &to_sock = to.inputs_[link.To().Index()];

  from_sock.Erase(to_sock);
  to_sock.Clear();
  --link_size_;
  // is_sorted_ = false; // NOTE: Erase a link does not affect the topology order.
}

CG_STRONG_INLINE bool Graph::HasCycle() const noexcept {
  return !nodes_.empty() && TopologyOrder().empty();
}

inline void Graph::ShrinkToFit() noexcept {
  node_container new_nodes;
  new_nodes.reserve(NumNodes());
  for (auto &node : nodes_) {
    if (node) {
      new_nodes.push_back(std::move(node));
    }
  }
  nodes_ = std::move(new_nodes);
  free_ids_.clear();
  RebuildAddrToIndex();
}

CG_STRONG_INLINE void Graph::RebuildAddrToIndex() noexcept {
  addr_to_index_.clear();
  for (size_t i = 0; i < nodes_.size(); ++i) {
    addr_to_index_[nodes_[i].get()] = i;
  }
}

CG_STRONG_INLINE void Graph::TopologySort() {
  if (is_sorted_) {
    return;
  }
  auto const order = TopologyOrder();
  node_container new_nodes;
  new_nodes.reserve(nodes_.size());
  for (size_t i : order) {
    new_nodes.push_back(std::move(nodes_[i]));
  }
  nodes_ = std::move(new_nodes);
  RebuildAddrToIndex();
  is_sorted_ = true;
}

CG_STRONG_INLINE std::vector<size_t> Graph::TopologyOrder() const noexcept {
  std::vector<size_t> result, empty_input;
  size_t const n = nodes_.size();
  result.reserve(n);
  std::vector<size_t> connected_count(n, 0);
  size_t empty = 0;
  for (size_t i = 0; i < n; ++i) {
    auto const &node = nodes_[i];
    if (!node) {
      result.push_back(i);
      ++empty;
      continue;
    }

    size_t count = 0;
    for (auto const &input : node->GetInputs()) {
      count += input.IsConnected() ? 1 : 0;
    }
    connected_count[i] = count;
    if (count == 0) {
      empty_input.push_back(i);
    }
  }
  std::copy(empty_input.begin(), empty_input.end(), std::back_inserter(result));

  for (size_t i = empty; i < result.size(); ++i) {
    auto const &node = nodes_[result[i]];
    assert(node != nullptr);
    for (auto const &output : node->GetOutputs()) {
      for (auto const *to_socket : output.ConnectedSockets()) {
        if (size_t const to_index = addr_to_index_.at(&(to_socket->Node()));
            --connected_count[to_index] == 0) {
          result.push_back(to_index);
        }
      }
    }
  }

  // for (size_t i = 0; i < n; ++i) {
  //   std::cout << i << " " << connected_count[i] << std::endl;
  // }
  // for (size_t i = 0; i < n; ++i) {
  //   std::cout << i << " " << result[i] << std::endl;
  // }

  if (result.size() == n) {
    return result;
  }
  return {};
}

}  // namespace compute_graph

// Helper macros to define a node.
#define CG_NODE_SOCKET_IMPL(ith, Type, Name, desc, ...)             \
  template <typename _WHATEVER> struct socket_meta<ith, _WHATEVER>  \
      : ::compute_graph::intern::SocketMetaBase {                   \
    using type = Type;                                              \
    static constexpr size_t index = ith;                            \
    static constexpr const char *name = #Name;                      \
    static constexpr const char *pretty_typename = #Type;           \
    static constexpr const char *description = desc;                \
    __VA_OPT__(static CG_STRONG_INLINE Type const &DefaultValue() { \
      static Type _v{__VA_ARGS__};                                  \
      return _v;                                                    \
    })                                                              \
  };                                                                \
  using Name##_ = socket_meta<ith, int>;                            \
  static constexpr Name##_ Name{};

#define CG_NODE_PP_ADAPTOR(x, i) \
  CG_PP_EVAL(CG_NODE_SOCKET_IMPL CG_PP_EMPTY()(i, CG_PP_TUPLE_UNPACK x))

// Usage:
// CG_NODE_INPUTS(
//    (<type>, <identifier>, <description>, <optional-default-value>),
//    (<type>, <identifier>, <description>, <optional-default-value>),
//    ...);
// Example:
// CG_NODE_INPUTS(
//    (int,         x, "integer input", 0 /* default = 0      */),
//    (std::string, y, "string input"     /* no default value */),
#define CG_NODE_INPUTS(...)                                             \
  typedef struct intern_input_meta {                                    \
    template <size_t I, typename = int> struct socket_meta {            \
      using type = void;                                                \
    };                                                                  \
    __VA_OPT__(CG_PP_VAOPT_FOR_EACH_I(CG_NODE_PP_ADAPTOR, __VA_ARGS__)) \
  } in

// Usage:
// CG_NODE_OUTPUTS(
//    (<type>, <identifier>, <description>),
//    (<type>, <identifier>, <description>),
//    ...);
#define CG_NODE_OUTPUTS(...)                                            \
  typedef struct intern_output_meta {                                   \
    template <size_t I, typename = int> struct socket_meta {            \
      using type = void;                                                \
    };                                                                  \
    __VA_OPT__(CG_PP_VAOPT_FOR_EACH_I(CG_NODE_PP_ADAPTOR, __VA_ARGS__)) \
  } out

#ifndef CG_MAYBE_UNUSED
#  define CG_MAYBE_UNUSED(x) (void)(x)
#endif

#ifdef CG_AUTO_REGISTER
#  define CG_NODE_REGISTER_BODY(NodeType)                              \
    struct intern_auto_register {                                      \
      CG_STRONG_INLINE explicit intern_auto_register(void *p) {        \
        CG_MAYBE_UNUSED(p);                                            \
        ::compute_graph::NodeRegistry::instance().DeferedLoad(         \
            [](::compute_graph::NodeRegistry &r) {                     \
              auto desc = NodeType::BuildDescriptor();                 \
              intern::call_on_register_if_presented<NodeType>::exec(); \
              r.Emplace(desc);                                         \
            });                                                        \
      }                                                                \
    };                                                                 \
    inline static const intern_auto_register intern_register{nullptr};
#else
#  define CG_NODE_REGISTER_BODY(NodeType) /* empty */
#endif

// Use to define a node.
#define CG_NODE_COMMON(NodeType, Name, Desc)                                                 \
  CG_STRONG_INLINE explicit NodeType(NodeDescriptor const &descriptor) noexcept              \
      : NodeDerive<NodeType>(descriptor) {                                                   \
    ::compute_graph::intern::call_on_construct_if_presented<NodeType>::exec(*this);          \
  }                                                                                          \
                                                                                             \
public:                                                                                      \
  friend class NodeDescriptorBuilder<NodeType>;                                              \
  static std::string name() { return (Name); }                                               \
  static std::string desc() { return (Desc); }                                               \
  CG_NODE_REGISTER_BODY(NodeType);                                                           \
  using NodeDerive<NodeType>::GetOr, NodeDerive<NodeType>::Get, NodeDerive<NodeType>::Has,   \
      NodeDerive<NodeType>::Set, NodeDerive<NodeType>::GetAll, NodeDerive<NodeType>::SetAll, \
      NodeDerive<NodeType>::DefaultValue, NodeDerive<NodeType>::Ensure
