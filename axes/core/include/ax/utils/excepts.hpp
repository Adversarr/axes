#pragma once
#include "ax/core/excepts.hpp"

namespace ax::utils {

class FildNotFoundError : public std::exception {
public:
  FildNotFoundError(const char* filename) : std::exception(filename) {}
};

class FildOpenError : public std::exception {
public:
FildOpenError(const char* filename) : std::exception(filename) {}
};

class FileReadError : public std::exception {
public:
  FileReadError(const char* filename) : std::exception(filename) {}
};

class FileWriteError : public std::exception {
public:
FileWriteError(const char* filename) : std::exception(filename) {}
};

}