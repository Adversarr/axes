#include "axes/core/ecs/world.hpp"

#include <absl/base/internal/prefetch.h>

#include "axes/core/init.hpp"
#include "axes/core/utils/log.hpp"

namespace axes::ecs {

struct RunningSystemInfo {
  std::shared_ptr<SystemBase> system_;
  bool operator<(const RunningSystemInfo&) const noexcept;
};

std::vector<RunningSystemInfo> systems_running_;

std::vector<std::shared_ptr<SystemBase>> systems_waiting_;

std::vector<std::shared_ptr<SystemBase>> systems_destroying_;

bool is_running_;

int return_value_;

std::vector<Event> events_;

EntityID counter_ = 0;

absl::flat_hash_set<EntityID> entities_{};

std::vector<ComponentManagerInfo> registered_components_{};

EntityID World::CreateEntity() {
  EntityID id = counter_;
  counter_++;
  entities_.insert(id);
  return id;
}

void World::DestroyEntity(EntityID entity) {
  if (entities_.contains(entity)) {
    entities_.erase(entity);
    for (const auto& man : registered_components_) {
      man.detach_(entity);
    }
  }
}

bool World::HasEntity(EntityID ent) { return entities_.contains(ent); }

std::vector<std::pair<std::type_index, void*>> World::GetEntityComponents(
    EntityID ent) {
  std::vector<std::pair<std::type_index, void*>> result;
  for (const auto& man : registered_components_) {
    void* ptr = static_cast<void*>(man.query_(ent));
    if (ptr) {
      result.push_back({man.ti_, ptr});
    }
  }
  return result;
}

const std::vector<ComponentManagerInfo>& World::GetRegisteredComponents() {
  return registered_components_;
}

void World::TryRegisterSystem(std::shared_ptr<SystemBase> sys) {
  systems_waiting_.emplace_back(std::move(sys));
}

void World::TryDestroySystem(std::shared_ptr<SystemBase> system) {
  // TODO: Implementation required.
  systems_destroying_.push_back(system);
}

int World::MainLoop(bool shutdown_axes) {
  AXES_INFO("World main loop start.");
  is_running_ = true;
  return_value_ = 0;
  while (is_running_) {
    PreLoop();
    LoopBody();
    // PostLoop();
  }

  if (shutdown_axes) {
    axes::shutdown();
  }
  return return_value_;
}

void World::PreLoop() {
  // Append Registered systems.
  for (auto& sys : systems_waiting_) {
    try {
      sys->Initialize();
      AXES_INFO("System \"{}\" added successfully. at [{:#08x}]", sys->GetName(),
                reinterpret_cast<size_t>(sys.get()));
      RunningSystemInfo rs;
      rs.system_.swap(sys);
      systems_running_.push_back(std::move(rs));
    } catch (const std::exception& except) {
      AXES_ERROR("Cannot initialize system \"{}\"at {:#08x}, message: \"{}\"",
                 sys->GetName(), reinterpret_cast<size_t>(sys.get()),
                 except.what());
    }
  }
  if (!systems_waiting_.empty()) {
    std::sort(systems_running_.begin(), systems_running_.end());
  }

  systems_waiting_.clear();
}

void World::LoopBody() {
  // Tick Logic.
  for (auto& run_sys : systems_running_) {
    run_sys.system_->TickLogic();
  }

  // Process Events.
  // TODO:all kinds of events should be processed here.
  for (auto event : events_) {
    if (event.GetKind() == EventKind::kSystemShutdown) {
      is_running_ = false;
    }
  }

  if (!is_running_) {
    return;
  }

  // Tick Render
  for (auto& run_sys : systems_running_) {
    run_sys.system_->TickRender();
  }
}

void World::EnqueueEvent(Event evt) { events_.push_back(evt); }

bool RunningSystemInfo::operator<(const RunningSystemInfo& rhs) const noexcept {
  return system_->GetPriority() < rhs.system_->GetPriority();
}

void World::DestroyAll() {
  for (const auto& c : registered_components_) {
    c.destroy_();
  }

  entities_.clear();
}

void World::RegisterComponent(ComponentManagerInfo info) {
  registered_components_.push_back(info);
}

const absl::flat_hash_set<EntityID>& World::GetEntities() { return entities_; }

}  // namespace axes::ecs
