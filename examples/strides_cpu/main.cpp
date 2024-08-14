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

  print_field<field1r>();
  print_field<field2r>();
  print_field<field3r>();
  print_field<field4r>();

  print_stl_vector<vec2r>();
  print_stl_vector<vec3r>();
  print_stl_vector<vec4r>();
  print_stl_vector<vec2i>();
  print_stl_vector<vec3i>();
  print_stl_vector<vec4i>();

  field4r f;
  Eigen::Block<Eigen::Matrix<double, 4, -1>, 4, 1, true> t = f.col(0);

  ax::clean_up();
  f.innerStride()
  return EXIT_SUCCESS;
}