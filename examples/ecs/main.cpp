//
// Created by Yang Jerry on 2023/5/11.
//

#include <acore/ecs/ecs.hpp>
#include <acore/utils/log.hpp>
#include <sstream>
#include "acore/init.hpp"

struct Vector3 {
  int x_, y_, z_;
  Vector3(int x, int y, int z) : x_{x}, y_{y}, z_{z} {
    std::cout << "Created an vector with " << ToString() << std::endl;
  }

  std::string ToString() const {
    std::ostringstream ss;
    ss << '[' << x_ << ", " << y_ << ", " << z_ << ']';
    return ss.str();
  }

  ~Vector3() { std::cout << "Vector " << ToString() << " gone." << std::endl; }
};

int main() {
  using namespace axes::ecs;

  axes::init_axes();
  AXES_INFO("Hi, this is ECS");
  // 1. You can create a world, and use it to create an entity.
  World world;
  EntityID ent0 = world.CreateEntity();
  // 1.1 or just use the static function:
  EntityID ent1 = World::CreateEntity();
  std::cout << "Entities created, id = [" << ent0 << ", " << ent1 << "]"
            << std::endl;

  std::cout << "World has no component for now: " << std::boolalpha
            << World::GetRegisteredComponents().empty() << std::endl;

  // 2. You can attach some components on them,
  ComponentManager<Vector3> manager;
  Vector3* comp_ptr = manager.AttachComponent(ent0, 1, 2, 3);
  std::cout << "Now ent0 has Vector3 " << comp_ptr->ToString() << std::endl;
  // 2.1 and now, you can know about the registered component.
  std::cout << "World has one component registered. name = "
            << world.GetRegisteredComponents().front().name() << std::endl;

  // 3. You can detach the component on the entity.
  manager.DetachComponent(ent0);
  std::cout << "ent0 has Vector3? " << std::boolalpha
            << (manager.Query(ent0) != nullptr);

  // 4. lets create some other components, and use an EntityManager.
  EntityManager em0{ent0}, em1{ent1};
  em0.Attach<std::string>("Entity 0").Attach<Vector3>(1, 2, 3);
  em1.Attach<std::string>("Entity 1");

  for (auto component : world.GetEntityComponents(ent0)) {
    std::cout << "Entity 0 has " << component.first.name() << " at "
              << component.second << std::endl;
  }

  for (auto component : world.GetEntityComponents(ent1)) {
    std::cout << "Entity 1 has " << component.first.name() << " at "
              << component.second << std::endl;
  }

  // 5. You can also get the component created by EntityManager.
  ComponentManager<std::string> name_manager;
  std::cout << "Entity 0 has name = " << *name_manager.Query(ent0) << std::endl;
  std::cout << "Entity 1 has name = " << *name_manager.Query(ent1) << std::endl;

  // 5.1 and modify them, or replace them directly.
  std::cout << "Entity 0 name cstr at "
            << (void*)name_manager.Query(ent0)->data() << std::endl;
  *name_manager.Query(ent0) = "Ent 0";
  std::cout << "Replace with new name "
            << (void*)name_manager.Query(ent0)->data() << std::endl;
  name_manager.ReplaceComponent(ent0, "Ent 01203124125");
  std::cout << "Replace component" << (void*)name_manager.Query(ent0)->data()
            << std::endl;

  // 6. reversely, you can get the entity that have the component.
  for (auto ent : name_manager.QueryAll()) {
    std::cout << "Entity " << ent << " obtains component `std::string` = "
              << *name_manager.Query(ent) << std::endl;
  }

  // 6.1 or directly use the iterator.
  for (auto [ent, ptr] : name_manager) {
    std::cout << "Entity " << ent
              << " obtains component `std::string` = " << *ptr << std::endl;
  }

  axes::shutdown_axes();
  return 0;
}
