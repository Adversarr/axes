#include <Eigen/Eigen>
#include <iostream>

auto main() -> int {
  using namespace Eigen;
  Vector3f a, b, c;
  a.setRandom();
  b.setRandom();
  c.setRandom();
  std::cout << a << std::endl;
  std::cout << b << std::endl;
  std::cout << c << std::endl;
  c.array() += a.array() * b.array();
  std::cout << c << std::endl;
  return 0;
}
