#include "ax/math/io.hpp"

#include <fstream>

#include "ax/core/excepts.hpp"

char numpy_magic_code[] = "\x93NUMPY";

namespace ax::math {

/*
Format Version 1.0

The first 6 bytes are a magic string: exactly \x93NUMPY.

The next 1 byte is an unsigned byte: the major version number of the file format, e.g. \x01.

The next 1 byte is an unsigned byte: the minor version number of the file format, e.g. \x00. Note:
the version of the file format is not tied to the version of the numpy package.

The next 2 bytes form a little-endian unsigned short int: the length of the header data HEADER_LEN.

The next HEADER_LEN bytes form the header data describing the array’s format. It is an ASCII string
which contains a Python literal expression of a dictionary. It is terminated by a newline (\n) and
padded with spaces (\x20) to make the total of len(magic string) + 2 + len(length) + HEADER_LEN be
evenly divisible by 64 for alignment purposes.

The dictionary contains three keys:

“descr”dtype.descr
An object that can be passed as an argument to the numpy.dtype constructor to create the array’s
dtype.

“fortran_order”bool
Whether the array data is Fortran-contiguous or not. Since Fortran-contiguous arrays are a common
form of non-C-contiguity, we allow them to be written directly to disk for efficiency.

“shape”tuple of int
The shape of the array.
For repeatability and readability, the dictionary keys are sorted in alphabetic order. This is for
convenience only. A writer SHOULD implement this if possible. A reader MUST NOT depend on this.

Following the header comes the array data. If the dtype contains Python objects (i.e.
dtype.hasobject is True), then the data is a Python pickle of the array. Otherwise the data is the
contiguous (either C- or Fortran-, depending on fortran_order) bytes of the array. Consumers can
figure out the number of bytes by multiplying the number of elements given by the shape (noting that
shape=() means there is 1 element) by dtype.itemsize.
*/

void write_npy_v10(std::ostream& out, const real* p, size_t write_length, size_t f, size_t i,
                   size_t j) {
  if (std::max<size_t>(f, 1) * std::max<size_t>(i, 1) * j != write_length) {
    throw std::runtime_error("The write length is not correct.");
  }
  out.write(numpy_magic_code, 6);
  uint8_t major = 1;
  uint8_t minor = 0;
  out.write(reinterpret_cast<char*>(&major), 1);
  out.write(reinterpret_cast<char*>(&minor), 1);

  uint16_t header_len = 0;
  std::ostringstream header_stream;
  header_stream << "{'descr': '<f8', 'fortran_order': False, 'shape': (";
  if (f > 0) {
    header_stream << f << ",";
  }
  if (i > 0) {
    header_stream << i << ",";
  }
  if (j > 0) {
    header_stream << j;
    if (f == 0 && i == 0) {
      header_stream << ",";
    }
  }
  header_stream << ")}";
  std::string header = header_stream.str();
  header_len = static_cast<uint16_t>(header.length());

  out.write(reinterpret_cast<char*>(&header_len), 2);
  out.write(header.c_str(), header_len);
  out.write(reinterpret_cast<const char*>(p), static_cast<long>(write_length * sizeof(real)));

  if (!out.good()) {
    // return void{voidCode::kUnavailable, "Failed to write to the ostream properly."};
    throw std::runtime_error("Failed to write to the ostream properly.");
  }
}

void write_npy_v10(std::ostream& out, const idx* p, size_t write_length, size_t f, size_t i,
                   size_t j) {
  if (std::max<size_t>(f, 1) * std::max<size_t>(i, 1) * j != write_length) {
    // return void{voidCode::kInvalidArgument, "The write length is not correct."};
    throw std::runtime_error("The write length is not correct.");
  }
  out.write(numpy_magic_code, 6);
  uint8_t major = 1;
  uint8_t minor = 0;
  out.write(reinterpret_cast<char*>(&major), 1);
  out.write(reinterpret_cast<char*>(&minor), 1);

  uint16_t header_len = 0;
  std::ostringstream header_stream;
  header_stream << "{'descr': '<i8', 'fortran_order': False, 'shape': (";
  if (f > 0) {
    header_stream << f << ",";
  }
  if (i > 0) {
    header_stream << i << ",";
  }
  if (j > 0) {
    header_stream << j;
    if (f == 0 && i == 0) {
      header_stream << ",";
    }
  }
  header_stream << ")}";
  std::string header = header_stream.str();
  header_len = static_cast<uint16_t>(header.length());

  out.write(reinterpret_cast<char*>(&header_len), 2);
  out.write(header.c_str(), header_len);
  out.write(reinterpret_cast<const char*>(p), static_cast<long>(write_length * sizeof(idx)));

  if (!out.good()) {
    // return void{voidCode::kUnavailable, "Failed to write to the ostream properly."};
    throw std::runtime_error("Failed to write to the ostream properly.");
  }
}

void write_npy_v10(std::string path, const vec<real, Eigen::Dynamic>& vec) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    // return void{voidCode::kInvalidArgument, "Failed to open the file."};
    throw std::runtime_error("Failed to open the file.");
  }

  write_npy_v10(out, vec.data(), static_cast<size_t>(vec.size()), 0, 0,
                static_cast<size_t>(vec.size()));
}

void write_npy_v10(std::string path, const vec<idx, Eigen::Dynamic>& vec) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    // return void{voidCode::kInvalidArgument, "Failed to open the file."};
    throw std::runtime_error("Failed to open the file: " + path);
  }

  write_npy_v10(out, vec.data(), static_cast<size_t>(vec.size()), 0, 0,
                static_cast<size_t>(vec.size()));
}

void write_npy_v10(std::string path, const mat<real, dynamic, dynamic>& mat) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    // void{voidCode::kInvalidArgument, "Failed to open the file."};
    throw std::runtime_error("Failed to open the file.");
  }

  std::vector<real> data;
  data.reserve(static_cast<size_t>(mat.cols() * mat.rows()));
  for (idx j = 0; j < mat.rows(); ++j) {
    for (idx i = 0; i < mat.cols(); ++i) {
      data.push_back(mat(j, i));
    }
  }

  write_npy_v10(out, data.data(), static_cast<size_t>(mat.size()), 0,
                static_cast<size_t>(mat.rows()), static_cast<size_t>(mat.cols()));
}

void write_npy_v10(std::string path, const mat<idx, dynamic, dynamic>& mat) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    // return void{voidCode::kInvalidArgument, "Failed to open the file."};
    throw std::runtime_error("Failed to open the file: " + path);
  }

  std::vector<idx> data;
  data.reserve(static_cast<size_t>(mat.cols() * mat.rows()));
  for (idx j = 0; j < mat.rows(); ++j) {
    for (idx i = 0; i < mat.cols(); ++i) {
      data.push_back(mat(j, i));
    }
  }

  write_npy_v10(out, data.data(), static_cast<size_t>(mat.size()), 0,
                static_cast<size_t>(mat.rows()), static_cast<size_t>(mat.cols()));
}

struct Header {
  std::string descr;
  bool fortran_order = false;
  std::vector<idx> shape;
  void parse(const std::string& header) {
    if (header.empty()) {
      // return void{voidCode::kInvalidArgument, "The header is empty."};
      throw std::runtime_error("The header is empty.");
    }

    std::stringstream ss(header);
    char c;
    // Need to escape the spaces
    auto consume_until = [&ss](char target) -> bool {
      char c;
      bool status = false;
      while ((status = static_cast<bool>(ss >> c))) {
        if (c == target) {
          break;
        }
      }
      return status;
    };
    if (!consume_until('{')) {
      // return utils::InvalidArgumentError("The header is not a valid dictionary.");
      throw std::runtime_error("The header is not a valid dictionary.");
    }

    while ((ss >> c)) {
      std::string key;
      std::string val;
      if (c == '\'') {
        while ((ss >> c)) {
          if (c == '\'') {
            break;
          }
          key.push_back(c);
        }
      }
      // std::cout << "Got " << key << std::endl;
      if (key.empty()) {
        break;
      }

      if (!consume_until(':')) {
        // return utils::InvalidArgumentError("The header is not a valid dictionary.");
        throw std::runtime_error("The header is not a valid dictionary.");
      }
      if (key == "descr") {
        if (!consume_until('\'')) {
          // return utils::InvalidArgumentError("The descr key is not a string.");
          throw std::runtime_error("The descr key is not a string.");
        }
        while (ss >> c) {
          if (c == '\'') {
            break;
          }
          val.push_back(c);
        }
        descr = val;
        // std::cout << "Descr: " << val << std::endl;
      } else if (key == "fortran_order") {
        while (ss >> c) {
          if (!std::isblank(c)) {
            break;
          }
        }
        if (c != 'T' && c != 'F') {
          // return utils::InvalidArgumentError("The fortran_order key is not a boolean.");
          throw std::runtime_error("The fortran_order key is not a boolean.");
        }
        fortran_order = c == 'T';
        // std::cout << "Fortran Order: " << fortran_order << std::endl;
      } else if (key == "shape") {
        while (ss >> c) {
          if (c == '(') {
            break;
          }
        }

        while (ss >> c) {
          if (c == ')') {
            ss.putback(')');
            break;
          }
          if (c == ',') {
            continue;
          }
          std::string num;
          num.push_back(c);
          while (ss >> c) {
            if (c != ')' && c != ',') {
              num.push_back(c);
            } else {
              break;
            }
          }
          shape.push_back(std::stoi(num));
          if (c == ')') break;
        }
        // for (auto i : shape) {
        //   std::cout << i << std::endl;
        // }
      }

      while (ss >> c) {
        // std::cout << "Consuming... " << c <<std::endl;
        if (c == '}' || c == ',') {
          break;
        }
      }
      if (c == '}') {
        // std::cout << "break!" << std::endl;
        break;
      }
      while (ss >> c) {
        if (c != ' ') {
          ss.putback(c);
          break;
        }
      }
    }
  }
};

math::matxxr read_npy_v10_real(std::string path) {
  std::ifstream in(path, std::ios::binary);
  // if (!in.is_open()) {
  //   throw FileNotFoundError(path);
  // }
  AX_THROW_IF_FALSE(in, "Failed to open the file: " + path);

  char magic[6];
  in.read(magic, 6);
  if (std::memcmp(magic, numpy_magic_code, 6) != 0) {
    throw RuntimeError("The file is not a valid NPY file. (magic code mismatch)");
  }

  uint8_t major, minor;
  in.read(reinterpret_cast<char*>(&major), 1);
  in.read(reinterpret_cast<char*>(&minor), 1);
  uint16_t header_len;
  in.read(reinterpret_cast<char*>(&header_len), 2);

  // Process Header
  std::string header;
  header.resize(header_len + 1, 0);
  in.read(header.data(), header_len);
  if (!in.good()) {
    throw RuntimeError("The file is not a valid NPY file. (Failed to read the header)");
  }
  // Read the data
  Header header_obj;
  header_obj.parse(header);
  if (header_obj.shape.size() > 2) {
    throw RuntimeError("dim > 2 is not supported");
  }
  idx rows = header_obj.shape[0];
  idx cols = header_obj.shape.size() > 1 ? header_obj.shape[1] : 1;
  math::matxxr mat(rows, cols);

  if (header_obj.descr[0] != '<') {
    throw RuntimeError("The data type is not little endianed.");
  }

  if (header_obj.descr[1] != 'f') {
    throw RuntimeError("The data type is not float.");
  }

  if (header_obj.descr[2] == '4') {
    if (header_obj.fortran_order) {
      for (idx i = 0; i < cols; ++i) {
        for (idx j = 0; j < rows; ++j) {
          float val;
          in.read(reinterpret_cast<char*>(&val), 4);
          mat(j, i) = static_cast<real>(val);
        }
      }
    } else {
      for (idx j = 0; j < rows; ++j) {
        for (idx i = 0; i < cols; ++i) {
          float val;
          in.read(reinterpret_cast<char*>(&val), 4);
          mat(j, i) = static_cast<real>(val);
        }
      }
    }
  } else if (header_obj.descr[2] == '8') {
    if (header_obj.fortran_order) {
      for (idx i = 0; i < cols; ++i) {
        for (idx j = 0; j < rows; ++j) {
          double val;
          in.read(reinterpret_cast<char*>(&val), 8);
          mat(j, i) = static_cast<real>(val);
        }
      }
    } else {
      for (idx j = 0; j < rows; ++j) {
        for (idx i = 0; i < cols; ++i) {
          double val;
          if (!in.read(reinterpret_cast<char*>(&val), 8)) {
            throw RuntimeError("Invalid npy file.");
          }
          mat(j, i) = static_cast<real>(val);
        }
      }
    }
  } else {
    throw RuntimeError("The data type is not float32 or float64.");
  }
  return mat;
}

math::matxxi read_npy_v10_idx(std::string path) {
  std::ifstream in(path, std::ios::binary);
  // if (!in.is_open()) {
  //   throw FileNotFoundError(path);
  // }
  AX_THROW_IF_FALSE(in, "Failed to open the file: " + path);

  char magic[6];
  in.read(magic, 6);
  if (std::memcmp(magic, numpy_magic_code, 6) != 0) {
    throw RuntimeError("The file is not a valid NPY file. (magic code mismatch)");
  }

  uint8_t major, minor;
  in.read(reinterpret_cast<char*>(&major), 1);
  in.read(reinterpret_cast<char*>(&minor), 1);
  uint16_t header_len;
  in.read(reinterpret_cast<char*>(&header_len), 2);

  // Process Header
  std::string header;
  header.resize(header_len + 1, 0);
  in.read(header.data(), header_len);
  if (!in.good()) {
    throw RuntimeError("The file is not a valid NPY file. (Failed to read the header)");
  }
  // Read the data
  Header header_obj;
  header_obj.parse(header);
  if (header_obj.shape.size() > 2) {
    throw RuntimeError("dim > 2 is not supported");
  }
  idx rows = header_obj.shape[0];
  idx cols = header_obj.shape.size() > 1 ? header_obj.shape[1] : 1;
  math::matxxi mat(rows, cols);

  if (header_obj.descr[0] != '<') {
    throw RuntimeError("The data type is not little endianed.");
  }

  if (header_obj.descr[1] != 'i') {
    throw RuntimeError("The data type is not long or int.");
  }

  if (header_obj.descr[2] == '4') {
    if (header_obj.fortran_order) {
      for (idx i = 0; i < cols; ++i) {
        for (idx j = 0; j < rows; ++j) {
          int val;
          in.read(reinterpret_cast<char*>(&val), 4);
          mat(j, i) = static_cast<idx>(val);
        }
      }
    } else {
      for (idx j = 0; j < rows; ++j) {
        for (idx i = 0; i < cols; ++i) {
          int val;
          in.read(reinterpret_cast<char*>(&val), 4);
          mat(j, i) = static_cast<idx>(val);
        }
      }
    }
  } else if (header_obj.descr[2] == '8') {
    if (header_obj.fortran_order) {
      for (idx i = 0; i < cols; ++i) {
        for (idx j = 0; j < rows; ++j) {
          int64_t val;
          in.read(reinterpret_cast<char*>(&val), 8);
          mat(j, i) = static_cast<idx>(val);
        }
      }
    } else {
      for (idx j = 0; j < rows; ++j) {
        for (idx i = 0; i < cols; ++i) {
          int64_t val;
          if (!in.read(reinterpret_cast<char*>(&val), 8)) {
            throw RuntimeError("Invalid npy file.");
          }
          mat(j, i) = static_cast<idx>(val);
        }
      }
    }
  } else {
    throw RuntimeError("The data type is not long or int");
  }
  return mat;
}

void write_sparse_matrix(std::string path, const spmatr& mat) {
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to open the file. " + path);
  }

  out << R"(%%MatrixMarket matrix coordinate real general
%=================================================================================
%
% This ASCII file represents a sparse MxN matrix with L 
% nonzeros in the following Matrix Market format:
%
% +----------------------------------------------+
% |%%MatrixMarket matrix coordinate real general | <--- header line
% |%                                             | <--+
% |% comments                                    |    |-- 0 or more comment lines
% |%                                             | <--+         
% |    M  N  L                                   | <--- rows, columns, entries
% |    I1  J1  A(I1, J1)                         | <--+
% |    I2  J2  A(I2, J2)                         |    |
% |    I3  J3  A(I3, J3)                         |    |-- L lines
% |        . . .                                 |    |
% |    IL JL  A(IL, JL)                          | <--+
% +----------------------------------------------+   
%
% Indices are 1-based, i.e. A(1,1) is the first element.
%
%=================================================================================)";

  out << "\n" << mat.rows() << " " << mat.cols() << " " << mat.nonZeros() << "\n";

  for (idx k = 0; k < mat.outerSize(); ++k) {
    for (spmatr::InnerIterator it(mat, k); it; ++it) {
      out << it.row() + 1 << " " << it.col() + 1 << " " << it.value() << "\n";
    }
  }
}

spmatr read_sparse_matrix(std::string path) {
  std::ifstream in(path);
  // if (!in.is_open()) {
  //   throw FileNotFoundError(path);
  // }
  AX_THROW_IF_FALSE(in, "Failed to open the file: " + path);
  char line[1024];
  in.getline(line, 1024);
  if (std::strncmp(line, "%%MatrixMarket matrix coordinate real general", 46) != 0) {
    throw RuntimeError("The file is not a valid MatrixMarket file.");
  }

  while (in.getline(line, 1024)) {
    if (line[0] != '%') {
      break;
    }
  }

  int rows, cols, nonzeros;
  idx n_success = std::sscanf(line, "%d %d %d", &rows, &cols, &nonzeros);
  if (n_success != 3) {
    throw RuntimeError("The file is not a valid MatrixMarket file.");
  }
  sp_coeff_list triplets;
  triplets.reserve(static_cast<size_t>(nonzeros));
  for (int i = 0; i < nonzeros; ++i) {
    if (!in.getline(line, 1024)) {
      throw RuntimeError("The file is not a valid MatrixMarket file.");
    }
    int r, c;
    real val;
    n_success = std::sscanf(line, "%d %d %lf", &r, &c, &val);
    if (n_success != 3) {
      throw RuntimeError(("line " + std::to_string(i) + " is not a valid triplet: ") + line);
    }
    triplets.push_back({r - 1, c - 1, val});
  }
  math::make_sparse_matrix(rows, cols, triplets);
}

}  // namespace ax::math
