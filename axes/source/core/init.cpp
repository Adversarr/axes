#include "axes/core/init.hpp"

#include <absl/debugging/failure_signal_handler.h>
#include <absl/debugging/symbolize.h>
#include <absl/flags/parse.h>
#include <absl/log/die_if_null.h>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/utils/common.hpp"

namespace ax {

/****************************** GLOBAL VARS ******************************/
utils::uptr<entt::registry> registry_p;

struct Hook {
  const char* name_;
  std::function<Status()> call_;
};

std::vector<Hook> init_hooks;
std::vector<Hook> clean_up_hooks;

/****************************** Implementation ******************************/

void init(int argc, char** argv) {
  using namespace absl;

  /****************************** Flags ******************************/
  absl::ParseCommandLine(argc, argv);

  /****************************** Install the debuggers ******************************/
  CHECK(argc > 0) << "argc must be greater than 0";
  absl::InitializeSymbolizer(argv[0]);
  FailureSignalHandlerOptions failure_signal_handler{};
  absl::InstallFailureSignalHandler(failure_signal_handler);

  /****************************** Setup Entt Registry ******************************/
  registry();

  /****************************** Run all the hooks ******************************/
  for (auto [name, call] : init_hooks) {
    CHECK_OK(call()) << "Init-hook [" << name << "] failed.";
  }
  init_hooks.clear();
}

void clean_up() {
  for (auto [name, call] : clean_up_hooks) {
    CHECK_OK(call()) << "CleanUp-hook [" << name << "] failed.";
  }
  registry_p->clear();
  clean_up_hooks.clear();
}

void hook_init(const char* name, std::function<Status()> f) { init_hooks.push_back({name, f}); }

void hook_clean_up(const char* name, std::function<Status()> f) {
  clean_up_hooks.push_back({name, f});
}

entt::registry& registry() {
  if (!registry_p) {
    registry_p = std::make_unique<entt::registry>();
  }
  return *registry_p;
}

}  // namespace ax
