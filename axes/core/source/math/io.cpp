#include "ax/math/io.hpp"

#include <fstream>

#include "ax/core/status.hpp"
#include "ax/utils/status.hpp"

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

Status write_npy_v10(std::ostream& out, const real* p, size_t write_length, size_t f, size_t i,
                     size_t j) {
  if (std::max<size_t>(f, 1) * std::max<size_t>(i, 1) * j != write_length) {
    return Status{StatusCode::kInvalidArgument, "The write length is not correct."};
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
  header_len = header.length();

  out.write(reinterpret_cast<char*>(&header_len), 2);
  out.write(header.c_str(), header_len);
  out.write(reinterpret_cast<const char*>(p), write_length * sizeof(real));

  if (!out.good()) {
    return Status{StatusCode::kUnavailable, "Failed to write to the ostream properly."};
  }
  return {};
}

Status write_npy_v10(std::string path, const vec<real, Eigen::Dynamic>& vec) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    return Status{StatusCode::kInvalidArgument, "Failed to open the file."};
  }

  return write_npy_v10(out, vec.data(), vec.size(), 0, 0, vec.size());
}

Status write_npy_v10(std::string path, const mat<real, dynamic, dynamic>& mat) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    return Status{StatusCode::kInvalidArgument, "Failed to open the file."};
  }

  List<real> data;
  data.reserve(mat.cols() * mat.rows());
  for (idx j = 0; j < mat.rows(); ++j) {
    for (idx i = 0; i < mat.cols(); ++i) {
      data.push_back(mat(j, i));
    }
  }

  return write_npy_v10(out, data.data(), mat.size(), 0, mat.rows(), mat.cols());
}

struct Header {
  std::string descr;
  bool fortran_order = false;
  std::vector<idx> shape;
  Status parse(const std::string& header) {
    if (header.empty()) {
      return Status{StatusCode::kInvalidArgument, "The header is empty."};
    }

    std::stringstream ss(header);
    char c;
    // Need to escape the spaces
    auto consume_until = [&ss](char target) -> bool {
      char c;
      bool status = false;
      while ((status = (bool) (ss >> c))) {
        if (c == target) {
          break;
        }
      }
      return status;
    };
    if (!consume_until('{')) {
      return utils::InvalidArgumentError("The header is not a valid dictionary.");
    }

    while (ss >> c) {
      std::string key;
      std::string val;
      if (c == '\'') {
        while (ss >> c) {
          if (c == '\'') {
            break;
          }
          key.push_back(c);
        }
      }
      std::cout << "Got " << key << std::endl;
      if (key.empty()) {
        break;
      }

      if (!consume_until(':')) {
        return utils::InvalidArgumentError("The header is not a valid dictionary.");
      }
      if (key == "descr") {
        if (!consume_until('\'')) {
          return utils::InvalidArgumentError("The descr key is not a string.");
        }
        while (ss >> c) {
          if (c == '\'') {
            break;
          }
          val.push_back(c);
        }
        descr = val;
        std::cout << "Descr: " << val << std::endl;
      } else if (key == "fortran_order") {
        while (ss >> c) {
          if (!std::isblank(c)) {
            break;
          }
        }
        if (c != 'T' && c != 'F') {
          return utils::InvalidArgumentError("The fortran_order key is not a boolean.");
        }
        fortran_order = c == 'T';
        std::cout << "Fortran Order: " << fortran_order << std::endl;
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
        for (auto i : shape) {
          std::cout << i << std::endl;
        }
      }

      while (ss >> c) {
        std::cout << "Consuming... " << c <<std::endl;
        if (c == '}' || c == ',') {
          break;
        }
      }
      if (c == '}') {
        std::cout << "break!" << std::endl;
        break;
      }
      while (ss >> c) {
        if (c != ' ') {
          ss.putback(c);
          break;
        }
      }
    }
    AX_RETURN_OK();
  }
};

StatusOr<math::matxxr> read_npy_v10(std::string path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    return utils::NotFoundError("Failed to open the file. " + path);
  }

  char magic[6];
  in.read(magic, 6);
  if (std::memcmp(magic, numpy_magic_code, 6) != 0) {
    return utils::FailedPreconditionError("The file is not a valid NPY file. (magic code mismatch)");
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
      return Status{StatusCode::kUnavailable, "Failed to read the header."};
  }
  // Read the data
  Header header_obj;
  if (auto s = header_obj.parse(header); !s.ok()) {
    return s;
  }

  if (header_obj.shape.size() > 2) {
    return utils::UnavailableError("The shape is larger than 2D");
  }
  idx rows = header_obj.shape[0];
  idx cols = header_obj.shape.size() > 1 ? header_obj.shape[1] : 1;
  math::matxxr mat(rows, cols);

  if (header_obj.descr[0] != '<') {
    return utils::UnavailableError("The data type is not little endianed.");
  }

  if (header_obj.descr[1] != 'f') {
    return utils::UnavailableError("The data type is not float.");
  }

  if (header_obj.descr[2] == '4') {
    if (header_obj.fortran_order) {
      for (idx i = 0; i < cols; ++i) {
        for (idx j = 0; j < rows; ++j) {
          float val;
          in.read(reinterpret_cast<char*>(&val), 4);
          mat(j, i) = (real) val;
        }
      }
    } else {
      for (idx j = 0; j < rows; ++j) {
        for (idx i = 0; i < cols; ++i) {
          float val;
          in.read(reinterpret_cast<char*>(&val), 4);
          mat(j, i) = (real)val;
        }
      }
    }
  } else if (header_obj.descr[2] == '8') {
    if (header_obj.fortran_order) {
      for (idx i = 0; i < cols; ++i) {
            for (idx j = 0; j < rows; ++j) {
              double val;
              in.read(reinterpret_cast<char*>(&val), 8);
              mat(j, i) = (real) val;
            }
      }
    } else {
      for (idx j = 0; j < rows; ++j) {
        for (idx i = 0; i < cols; ++i) {
          double val;
          if (!in.read(reinterpret_cast<char*>(&val), 8)) {
            return utils::FailedPreconditionError("Invalid npy file.");
          }
          mat(j, i) = (real) val;
        }
      }
    }
  } else {
    return utils::UnavailableError("The data type is not float32 or float64.");
  }
  return mat;
}


}  // namespace ax::math