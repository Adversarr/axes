#include "axes/math/io.hpp"

#include <fstream>

#include "axes/core/status.hpp"

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

  return write_npy_v10(out, mat.data(), mat.size(), 0, mat.rows(), mat.cols());
}

}  // namespace ax::math