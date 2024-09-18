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

void write_npy_v10(std::ostream& out, const Real* p, size_t write_length, size_t f, size_t i,
                   size_t j) {
  if (std::max<size_t>(f, 1) * std::max<size_t>(i, 1) * j != write_length) {
    AX_THROW_RUNTIME_ERROR("The write length is not correct.");
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
  out.write(reinterpret_cast<const char*>(p), static_cast<long>(write_length * sizeof(Real)));

  if (!out.good()) {
    AX_THROW_RUNTIME_ERROR("Failed to write to the ostream properly.");
  }
}

static void write_npy_v10(std::ostream& out, const Index* p, size_t write_length, size_t f, size_t i,
                          size_t j) {
  if (std::max<size_t>(f, 1) * std::max<size_t>(i, 1) * j != write_length) {
    AX_THROW_RUNTIME_ERROR("The write length is not correct.");
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
  out.write(reinterpret_cast<const char*>(p),
            static_cast<std::streamsize>(write_length * sizeof(Index)));

  if (!out.good()) {
    AX_THROW_RUNTIME_ERROR("Failed to write to the ostream properly.");
  }
}

void write_npy_v10(std::string path, const Vector<Real, Eigen::Dynamic>& vec) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    // return void{voidCode::kInvalidArgument, "Failed to open the file."};
    AX_THROW_RUNTIME_ERROR("Failed to open the file.");
  }

  write_npy_v10(out, vec.data(), static_cast<size_t>(vec.size()), 0, 0,
                static_cast<size_t>(vec.size()));
}

void write_npy_v10(std::string path, const Vector<Index, Eigen::Dynamic>& vec) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    // return void{voidCode::kInvalidArgument, "Failed to open the file."};
    AX_THROW_RUNTIME_ERROR("Failed to open the file: {}", path);
  }

  write_npy_v10(out, vec.data(), static_cast<size_t>(vec.size()), 0, 0,
                static_cast<size_t>(vec.size()));
}

void write_npy_v10(std::string path, const Matrix<Real, dynamic, dynamic>& mat) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    // void{voidCode::kInvalidArgument, "Failed to open the file."};
    AX_THROW_RUNTIME_ERROR("Failed to open the file.");
  }

  std::vector<Real> data;
  data.reserve(static_cast<size_t>(mat.cols() * mat.rows()));
  for (Index j = 0; j < mat.rows(); ++j) {
    for (Index i = 0; i < mat.cols(); ++i) {
      data.push_back(mat(j, i));
    }
  }

  write_npy_v10(out, data.data(), static_cast<size_t>(mat.size()), 0,
                static_cast<size_t>(mat.rows()), static_cast<size_t>(mat.cols()));
}

void write_npy_v10(std::string path, const Matrix<Index, dynamic, dynamic>& mat) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    // return void{voidCode::kInvalidArgument, "Failed to open the file."};
    AX_THROW_RUNTIME_ERROR("Failed to open the file: {}", path);
  }

  std::vector<Index> data;
  data.reserve(static_cast<size_t>(mat.cols() * mat.rows()));
  for (Index j = 0; j < mat.rows(); ++j) {
    for (Index i = 0; i < mat.cols(); ++i) {
      data.push_back(mat(j, i));
    }
  }

  write_npy_v10(out, data.data(), static_cast<size_t>(mat.size()), 0,
                static_cast<size_t>(mat.rows()), static_cast<size_t>(mat.cols()));
}

struct Header {
  std::string descr_;
  bool fortran_order_ = false;
  std::vector<Index> shape_;
  void Parse(const std::string& header) {
    if (header.empty()) {
      AX_THROW_RUNTIME_ERROR("The header is empty.");
    }

    std::stringstream ss(header);
    char c;
    auto consume_until = [&ss](char target) -> bool {
      char c = '\0';
      bool status = false;
      while ((status = static_cast<bool>(ss >> c))) {
        if (c == target) {
          break;
        }
      }
      return status;
    };
    if (!consume_until('{')) {
      AX_THROW_RUNTIME_ERROR("The header is not a valid dictionary.");
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
      if (key.empty()) {
        break;
      }

      if (!consume_until(':')) {
        AX_THROW_RUNTIME_ERROR("The header is not a valid dictionary.");
      }
      if (key == "descr") {
        if (!consume_until('\'')) {
          AX_THROW_RUNTIME_ERROR("The descr key is not a string.");
        }
        while (ss >> c) {
          if (c == '\'') {
            break;
          }
          val.push_back(c);
        }
        descr_ = val;
      } else if (key == "fortran_order") {
        while (ss >> c) {
          if (!std::isblank(c)) {
            break;
          }
        }
        if (c != 'T' && c != 'F') {
          AX_THROW_RUNTIME_ERROR("The fortran_order key is not a boolean.");
        }
        fortran_order_ = c == 'T';
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
          shape_.push_back(std::stoi(num));
          if (c == ')') break;
        }
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

math::RealMatrixX read_npy_v10_real(std::string path) {
  std::ifstream in(path, std::ios::binary);
  AX_THROW_IF_FALSE(in, "Failed to open the file: {}", path);

  char magic[6];
  in.read(magic, 6);
  if (std::memcmp(magic, numpy_magic_code, 6) != 0) {
    AX_THROW_RUNTIME_ERROR("The file is not a valid NPY file. (magic code mismatch)");
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
    AX_THROW_RUNTIME_ERROR("The file is not a valid NPY file. (Failed to read the header)");
  }
  // Read the data
  Header header_obj;
  header_obj.Parse(header);
  if (header_obj.shape_.size() > 2) {
    AX_THROW_RUNTIME_ERROR("dim > 2 is not supported");
  }
  Index rows = header_obj.shape_[0];
  Index cols = header_obj.shape_.size() > 1 ? header_obj.shape_[1] : 1;
  math::RealMatrixX mat(rows, cols);

  if (header_obj.descr_[0] != '<') {
    AX_THROW_RUNTIME_ERROR("The data type is not little endianed.");
  }

  if (header_obj.descr_[1] != 'f') {
    AX_THROW_RUNTIME_ERROR("The data type is not float.");
  }

  if (header_obj.descr_[2] == '4') {
    if (header_obj.fortran_order_) {
      for (Index i = 0; i < cols; ++i) {
        for (Index j = 0; j < rows; ++j) {
          float val;
          in.read(reinterpret_cast<char*>(&val), 4);
          mat(j, i) = static_cast<Real>(val);
        }
      }
    } else {
      for (Index j = 0; j < rows; ++j) {
        for (Index i = 0; i < cols; ++i) {
          float val;
          in.read(reinterpret_cast<char*>(&val), 4);
          mat(j, i) = static_cast<Real>(val);
        }
      }
    }
  } else if (header_obj.descr_[2] == '8') {
    if (header_obj.fortran_order_) {
      for (Index i = 0; i < cols; ++i) {
        for (Index j = 0; j < rows; ++j) {
          double val;
          in.read(reinterpret_cast<char*>(&val), 8);
          mat(j, i) = static_cast<Real>(val);
        }
      }
    } else {
      for (Index j = 0; j < rows; ++j) {
        for (Index i = 0; i < cols; ++i) {
          double val;
          if (!in.read(reinterpret_cast<char*>(&val), 8)) {
            AX_THROW_RUNTIME_ERROR("Invalid npy file.");
          }
          mat(j, i) = static_cast<Real>(val);
        }
      }
    }
  } else {
    AX_THROW_RUNTIME_ERROR("The data type is not float32 or float64.");
  }
  return mat;
}

math::IndexMatrixX read_npy_v10_Index(std::string path) {
  std::ifstream in(path, std::ios::binary);
  // if (!in.is_open()) {
  //   throw FileNotFoundError(path);
  // }
  AX_THROW_IF_FALSE(in, "Failed to open the file: ", path);

  char magic[6];
  in.read(magic, 6);
  if (std::memcmp(magic, numpy_magic_code, 6) != 0) {
    AX_THROW_RUNTIME_ERROR("The file is not a valid NPY file. (magic code mismatch)");
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
    AX_THROW_RUNTIME_ERROR("The file is not a valid NPY file. (Failed to read the header)");
  }
  // Read the data
  Header header_obj;
  header_obj.Parse(header);
  if (header_obj.shape_.size() > 2) {
    AX_THROW_RUNTIME_ERROR("dim > 2 is not supported");
  }
  Index rows = header_obj.shape_[0];
  Index cols = header_obj.shape_.size() > 1 ? header_obj.shape_[1] : 1;
  math::IndexMatrixX mat(rows, cols);

  if (header_obj.descr_[0] != '<') {
    AX_THROW_RUNTIME_ERROR("The data type is not little endianed.");
  }

  if (header_obj.descr_[1] != 'i') {
    AX_THROW_RUNTIME_ERROR("The data type is not long or int.");
  }

  if (header_obj.descr_[2] == '4') {
    if (header_obj.fortran_order_) {
      for (Index i = 0; i < cols; ++i) {
        for (Index j = 0; j < rows; ++j) {
          int val;
          in.read(reinterpret_cast<char*>(&val), 4);
          mat(j, i) = static_cast<Index>(val);
        }
      }
    } else {
      for (Index j = 0; j < rows; ++j) {
        for (Index i = 0; i < cols; ++i) {
          int val;
          in.read(reinterpret_cast<char*>(&val), 4);
          mat(j, i) = static_cast<Index>(val);
        }
      }
    }
  } else if (header_obj.descr_[2] == '8') {
    if (header_obj.fortran_order_) {
      for (Index i = 0; i < cols; ++i) {
        for (Index j = 0; j < rows; ++j) {
          int64_t val;
          in.read(reinterpret_cast<char*>(&val), 8);
          mat(j, i) = static_cast<Index>(val);
        }
      }
    } else {
      for (Index j = 0; j < rows; ++j) {
        for (Index i = 0; i < cols; ++i) {
          int64_t val;
          if (!in.read(reinterpret_cast<char*>(&val), 8)) {
            AX_THROW_RUNTIME_ERROR("Invalid npy file.");
          }
          mat(j, i) = static_cast<Index>(val);
        }
      }
    }
  } else {
    AX_THROW_RUNTIME_ERROR("The data type is not long or int");
  }
  return mat;
}

void write_sparse_matrix(std::string path, const RealSparseMatrix& mat) {
  std::ofstream out(path);
  if (!out.is_open()) {
    AX_THROW_RUNTIME_ERROR("Failed to open the file: {}", path);
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

  for (Index k = 0; k < mat.outerSize(); ++k) {
    for (RealSparseMatrix::InnerIterator it(mat, k); it; ++it) {
      out << it.row() + 1 << " " << it.col() + 1 << " " << it.value() << "\n";
    }
  }
}

RealSparseMatrix read_sparse_matrix(std::string path) {
  std::ifstream in(path);
  AX_THROW_IF_FALSE(in, "Failed to open the file: {}", path);
  char line[1024];
  in.getline(line, 1024);
  if (std::strncmp(line, "%%MatrixMarket matrix coordinate real general", 46) != 0) {
    AX_THROW_RUNTIME_ERROR("The file is not a valid MatrixMarket file.");
  }

  while (in.getline(line, 1024)) {
    if (line[0] != '%') {
      break;
    }
  }

  int rows, cols, nonzeros;
  Index n_success = std::sscanf(line, "%d %d %d", &rows, &cols, &nonzeros);
  if (n_success != 3) {
    AX_THROW_RUNTIME_ERROR("The file is not a valid MatrixMarket file.");
  }
  RealSparseCOO triplets;
  triplets.reserve(static_cast<size_t>(nonzeros));
  for (int i = 0; i < nonzeros; ++i) {
    if (!in.getline(line, 1024)) {
      AX_THROW_RUNTIME_ERROR("The file is not a valid MatrixMarket file.");
    }
    int r, c;
    Real val;
    n_success = std::sscanf(line, "%d %d %lf", &r, &c, &val);
    if (n_success != 3) {
      AX_THROW_RUNTIME_ERROR("Line {} is not a valid triplet: {}", i, line);
    }
    triplets.push_back({r - 1, c - 1, val});
  }
  return math::make_sparse_matrix(rows, cols, triplets);
}

}  // namespace ax::math
