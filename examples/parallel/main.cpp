#include "ax/core/buffer/for_each.hpp"
#include "ax/core/buffer/host_buffer.hpp"
#include "ax/core/init.hpp"
#include "ax/core/parallel/launch.hpp"
#include "ax/math/accessor.hpp"
#include "ax/math/views.hpp"

using namespace ax;

void run_empty() {
  parallel::TfBuilder builder;
  auto flow = builder.Build();
  auto executor = parallel::make_executor();
  executor.run(flow).wait();
}

void run_for_each_index() {
  parallel::TfBuilder builder;
  std::vector<size_t> v(8 * 24, 0);

  builder.ForEachIndex(
      parallel::ExecParam<3, 3>{{2, 2, 2}, {2, 3, 4}}, [&](Dim<3> grid_id, Dim<3> block_id) {
        auto grid_linear = parallel::details::linear_index(Dim<3>{2, 2, 2}, grid_id);
        auto block_linear = parallel::details::linear_index(Dim<3>{2, 3, 4}, block_id);
        v[grid_linear * 24 + block_linear] = grid_linear * 24 + block_linear;
      });
  auto flow = builder.Build();
  auto executor = parallel::make_executor();
  executor.run(flow).wait();

  for (size_t i = 0; i < v.size(); ++i) {
    AX_CHECK(v[i] == i, "Error for_each_index!");
  }
}

void run_for_each_buffer() {
  parallel::TfBuilder builder;
  auto buf = HostBuffer<int>::Create({8, 3, 4});
  std::atomic_int count = 0;
  builder.ForEach3D(std::tuple{buf->View(), buf->View()}, [&](int& v, int& /* also v */) {
    auto this_count = count.fetch_add(1);
    v = this_count;
  });

  auto flow = builder.Build();
  auto executor = parallel::make_executor();
  executor.run(flow).wait();
  std::set<int> all;
  for_each(buf->View(), [&](int v) {
    all.insert(v);
  });

  AX_CHECK(all.size() == 8 * 3 * 4, "Error for_each_buffer!");
}

void run_dot_product() {
  auto buf1 = HostBuffer<math::RealVector3>::Create({8, 3, 4});
  auto buf2 = HostBuffer<math::RealVector3>::Create({8, 3, 4});
  auto executor = parallel::make_executor();
  parallel::TfBuilder builder;

  for_each(make_view(buf1, buf2), [](auto& v, auto& w) {
    v.setOnes();
    w.setOnes();
  });

  auto set_ones = builder.ForEachIndex(parallel::ExecParam<3, 3>{{8, 3, 4}, {1, 1, 1}},
                                       [&](Dim<3> grid_id, Dim<3> block_id) {
                                         auto [i, j, k] = grid_id.sizes_;
                                         buf1->View()(i, j, k) = math::RealVector3::Ones();
                                         buf2->View()(i, j, k) = math::RealVector3::Ones();
                                       });
  auto echo = builder.Emplace([]() {
    AX_INFO("Init ones done.");
  });

  Real dot_prod = 0;
  auto dot = builder.TransformReduceIndex<Real>(
      parallel::ExecParam<3, 3>{{8, 3, 4}, {1, 1, 1}}, dot_prod,
      [b1 = buf1->View(), b2 = buf2->View()](const Dim<3>& g, const Dim<3>& b) {
        auto [x, y, z] = g.sizes_;
        return math::dot(b1(x, y, z), b2(x, y, z));
      },
      [](Real a, Real b) {
        return a + b;
      });
  set_ones.precede(echo);
  echo.precede(dot);

  auto flow = builder.Build();
  executor.run(flow).wait();
  AX_CHECK(dot_prod == 8 * 3 * 4 * 3, "Error dot product!");
}

void run_dot_product_buf() {
  auto buf1 = HostBuffer<math::RealVector3>::Create({8, 3, 4});
  auto buf2 = HostBuffer<math::RealVector3>::Create({8, 3, 4});
  auto executor = parallel::make_executor();
  parallel::TfBuilder builder;

  for_each(make_view(buf1, buf2), [](auto& v, auto& w) {
    v.setOnes();
    w.setOnes();
  });

  Real dot_prod = 0;
  builder.TransformReduce<Real>(
      make_view(buf1, buf2), dot_prod,
      [](math::RealVector3& x, math::RealVector3& y) {
        return math::dot(x, y);
      },
      [](Real a, Real b) -> Real{
        return a + b;
      });
  auto flow = builder.Build();
  auto dot_task = executor.run(flow);
  dot_task.wait();
  AX_CHECK(dot_prod == 8 * 3 * 4 * 3, "Error dot product!");
}

#define RUN(func)               \
  do {                          \
    AX_INFO("Run: {}", #func);  \
    func();                     \
    AX_INFO("Done: {}", #func); \
  } while (false);

int main(int argc, char** argv) {
  ax::initialize(argc, argv);
  RUN(run_empty);
  RUN(run_for_each_index);
  RUN(run_for_each_buffer);
  RUN(run_dot_product);
  RUN(run_dot_product_buf);
  auto hb = HostBuffer<float>::Create({3, 2, 1});
  for_each(hb->View(), [x = 0.f](float& v) mutable {
    v = x;
    x += 0.1f;
  });
  for (auto& b : hb->View()) {
    AX_INFO("{}", b);
  }

  auto hb_a = math::make_buf_accessor_3d(hb->ConstView());
  for (auto [i, b] : math::enumerate(hb_a)) {
    AX_INFO("{} {} {}: {}", i[0], i[1], i[2], b);
  }
  ax::clean_up();
  return EXIT_SUCCESS;
}