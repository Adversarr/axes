#pragma once

#include <exception>
#include <functional>
#include <optional>
#include <variant>
#include <memory>
#include <stdexcept>
#include <typeinfo>

namespace ax {

using BadCast = std::bad_cast;
using BadTypeid = std::bad_typeid;
using BadWeakPtr = std::bad_weak_ptr;
using BadFunctionCall = std::bad_function_call;
using BadAlloc = std::bad_alloc;
using BadArrayNewLength = std::bad_array_new_length;
using BadException = std::bad_exception;
using BadOptionalAccess = std::bad_optional_access;
using BadVariantAccess = std::bad_variant_access;
using InvalidArgument = std::invalid_argument;
using OutOfRange = std::out_of_range;
using RuntimeError = std::runtime_error;
using LogicError = std::logic_error;
using LengthError = std::length_error;
using RangeError = std::range_error;
using OverflowError = std::overflow_error;
using UnderflowError = std::underflow_error;
using DomainError = std::domain_error;

}