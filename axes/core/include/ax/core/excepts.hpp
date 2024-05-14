#pragma once

#include <exception>
#include <functional>
#include <optional>
#include <variant>
#include <memory>
#include <stdexcept>
#include <typeinfo>

#include <boost/preprocessor/facilities/va_opt.hpp>

#include "ax/core/macros.hpp"

namespace ax {

using BadCast = std::bad_cast;
using BadTypeid = std::bad_typeid;
// using BadWeakPtr = std::bad_weak_ptr;
// using BadFunctionCall = std::bad_function_call;
using BadAlloc = std::bad_alloc;
using BadArrayNewLength = std::bad_array_new_length;
using BadException = std::bad_exception;
// using BadOptionalAccess = std::bad_optional_access;
// using BadVariantAccess = std::bad_variant_access;
using InvalidArgument = std::invalid_argument;
using OutOfRange = std::out_of_range;
using RuntimeError = std::runtime_error;
using LogicError = std::logic_error;
using LengthError = std::length_error;
using RangeError = std::range_error;
using OverflowError = std::overflow_error;
using UnderflowError = std::underflow_error;
using DomainError = std::domain_error;

}  // namespace ax

#ifdef AX_PLATFORM_WINDOWS
#define AX_THROW_IF_LT(lhs, rhs, ...) do { if ((lhs) < (rhs)) throw ax::RuntimeError(#lhs " < " #rhs ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_GT(lhs, rhs, ...) do { if ((lhs) > (rhs)) throw ax::RuntimeError(#lhs " > " #rhs ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_LE(lhs, rhs, ...) do { if ((lhs) <= (rhs)) throw ax::RuntimeError(#lhs " <= " #rhs ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_GE(lhs, rhs, ...) do { if ((lhs) >= (rhs)) throw ax::RuntimeError(#lhs " >= " #rhs ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_EQ(lhs, rhs, ...) do { if ((lhs) == (rhs)) throw ax::RuntimeError(#lhs " == " #rhs ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_NE(lhs, rhs, ...) do { if ((lhs) != (rhs)) throw ax::RuntimeError(#lhs " != " #rhs ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_NULL(ptr, ...) do { if (!(ptr)) throw ax::RuntimeError(#ptr " is null" ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_NOT_NULL(ptr, ...) do { if ((ptr)) throw ax::RuntimeError(#ptr " is not null" ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_FALSE(cond, ...) do { if (!(cond)) throw ax::RuntimeError(#cond " is false" ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_TRUE(cond, ...) do { if ((cond)) throw ax::RuntimeError(#cond " is true" ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_NULLPTR(ptr, ...) do { if ((ptr) == nullptr) throw ax::RuntimeError(#ptr " is nullptr" ": " __VA_ARGS__); } while (0)
#define AX_THROW_IF_NOT_NULLPTR(ptr, ...) do { if ((ptr) != nullptr) throw ax::RuntimeError(#ptr " is not nullptr" ": " __VA_ARGS__); } while (0)
#else
#define AX_THROW_IF_LT(lhs, rhs, ...) do { if ((lhs) < (rhs)) throw ax::RuntimeError(#lhs " < " #rhs __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_GT(lhs, rhs, ...) do { if ((lhs) > (rhs)) throw ax::RuntimeError(#lhs " > " #rhs __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_LE(lhs, rhs, ...) do { if ((lhs) <= (rhs)) throw ax::RuntimeError(#lhs " <= " #rhs __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_GE(lhs, rhs, ...) do { if ((lhs) >= (rhs)) throw ax::RuntimeError(#lhs " >= " #rhs __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_EQ(lhs, rhs, ...) do { if ((lhs) == (rhs)) throw ax::RuntimeError(#lhs " == " #rhs __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_NE(lhs, rhs, ...) do { if ((lhs) != (rhs)) throw ax::RuntimeError(#lhs " != " #rhs __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_NULL(ptr, ...) do { if (!(ptr)) throw ax::RuntimeError(#ptr " is null" __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_NOT_NULL(ptr, ...) do { if ((ptr)) throw ax::RuntimeError(#ptr " is not null" __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_FALSE(cond, ...) do { if (!(cond)) throw ax::RuntimeError(#cond " is false" __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_TRUE(cond, ...) do { if ((cond)) throw ax::RuntimeError(#cond " is true" __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_NULLPTR(ptr, ...) do { if ((ptr) == nullptr) throw ax::RuntimeError(#ptr " is nullptr" __VA_OPT__(": ") __VA_ARGS__); } while (0)
#define AX_THROW_IF_NOT_NULLPTR(ptr, ...) do { if ((ptr) != nullptr) throw ax::RuntimeError(#ptr " is not nullptr" __VA_OPT__(": ") __VA_ARGS__); } while (0)
#endif
