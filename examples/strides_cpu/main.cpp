#include <ax/math/common.hpp>

#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/field.hpp"
#include "ax/utils/pretty_typename.hpp"

using namespace ax::math;

template <typename Container> void print_field() {
  Container c(Container::RowsAtCompileTime, 2);

  void* ptr = &c.col(0).x();
  void* ptr1 = &c.col(1).x();
  char* ptr_char = reinterpret_cast<char*>(ptr);
  char* ptr1_char = reinterpret_cast<char*>(ptr1);

  AX_INFO("Container={},  ptr_diff={}",
      ax::utils::pretty_type_name<Container>(),
      ptr1_char - ptr_char);
}

template <typename T> void print_stl_vector() {
  std::vector<T> c;

  void* ptr = &c[0];
  void* ptr1 = &c[1];
  char* ptr_char = reinterpret_cast<char*>(ptr);
  char* ptr1_char = reinterpret_cast<char*>(ptr1);

  AX_INFO("T={},  ptr_diff={}, sizeof(T)={}",
      ax::utils::pretty_type_name<T>(),
      ptr1_char - ptr_char,
      sizeof(T));
}


int main(int argc, char** argv) {
  ax::init(argc, argv);

  print_field<RealField1>();
  print_field<RealField2>();
  print_field<RealField3>();
  print_field<RealField4>();

  print_stl_vector<RealVector2>();
  print_stl_vector<RealVector3>();
  print_stl_vector<RealVector4>();
  print_stl_vector<IndexVec2>();
  print_stl_vector<IndexVec3>();
  print_stl_vector<IndexVec4>();

  RealField4 f;
  Eigen::Block<Eigen::Matrix<double, 4, -1>, 4, 1, true> t = f.col(0);

  ax::clean_up();
  f.innerStride()
  return EXIT_SUCCESS;
}