#include <doctest/doctest.h>

#include <axes/core/ecs/chunk.hpp>
#include <axes/core/ecs/component_manager.hpp>
#include <axes/core/ecs/ecs.hpp>
#include <axes/core/ecs/entity_manager.hpp>
#include <axes/core/ecs/world.hpp>
#include <iostream>

struct EchoCD {
  EchoCD() {
    std::cout << "Constructing Echo (address=" << this << ")" << std::endl;
  }
  ~EchoCD() {
    std::cout << "Destroying Echo (address=" << this << ")" << std::endl;
  }
};

TEST_CASE("ComponentManager") {
  axes::ecs::ComponentManager<int> integer_manager;
  int* e0c0 = integer_manager.AttachComponent(0, 1);
  int* e1c0 = integer_manager.AttachComponent(1, 1);
  CHECK(integer_manager.QueryAll().size() == 2);
  axes::ecs::ComponentManager<EchoCD> test_man;
  auto* cid = test_man.AttachComponent(0);
  test_man.AttachComponent(1);
  test_man.DetachComponent(0);
  auto* cid2 = test_man.AttachComponent(2);
  for (int i = 0; i < 120; ++i) {
    integer_manager.AttachComponent(i, i);
  }
  CHECK(cid == cid2);
  CHECK(integer_manager.QueryAll().size() == 122);

  auto* ptr = integer_manager.Query(10);
  CHECK(ptr != nullptr);
  CHECK(*ptr == 10);
  integer_manager.DetachComponent(10);
  ptr = integer_manager.Query(10);
  CHECK(ptr == nullptr);

  axes::ecs::ComponentManager<int> iman;
  CHECK(iman.Query(10) == nullptr);
  CHECK(iman.QueryAll().size() == 121);

  axes::ecs::World world;
  auto comps = world.GetRegisteredComponents();
  CHECK(comps.size() == 2);

  for (auto k : comps) {
    std::cout << k.ti_.name() << std::endl;
  }
}

TEST_CASE("ecs") {
  using namespace axes::ecs;
  EntityManager manager(World::CreateEntity());
  CHECK(manager.GetEntityID() == 0);
  manager.Attach<int>(10);
  auto* int_cptr = ComponentManager<int>{}.Query(0);
  CHECK_EQ(*int_cptr, 10);
  manager.Attach<EchoCD>();
  manager.Attach<std::string>("Name is hello");
  std::cout << *(ComponentManager<std::string>{}.Query(0)) << std::endl;

  for (auto val : World::GetEntityComponents(manager.GetEntityID())) {
    std::cout << val.first.name() << std::endl;
  }
}
