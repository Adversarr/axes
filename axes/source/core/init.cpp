#include "axes/core/init.hpp"
#include <absl/log/initialize.h>
#include <absl/debugging/failure_signal_handler.h>
#include <absl/debugging/symbolize.h>
#include <absl/flags/parse.h>
#include <absl/log/log.h>
#include <absl/log/globals.h>
#include <absl/log/die_if_null.h>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/utils/common.hpp"

namespace ax {

/****************************** GLOBAL VARS ******************************/
utils::uptr<entt::registry> registry_p;
utils::uptr<entt::dispatcher> dispatcher_p;

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
  absl::InitializeLog();
  absl::SetMinLogLevel(absl::LogSeverityAtLeast::kInfo);
  absl::SetStderrThreshold(absl::LogSeverity::kInfo);
  /****************************** Install the debuggers ******************************/
  AX_CHECK(argc > 0) << "argc must be greater than 0";
  absl::InitializeSymbolizer(argv[0]);
  FailureSignalHandlerOptions failure_signal_handler{};
  absl::InstallFailureSignalHandler(failure_signal_handler);

  /****************************** Setup Entt Registry ******************************/
  registry_p = std::make_unique<entt::registry>();
  dispatcher_p = std::make_unique<entt::dispatcher>();

  /****************************** Run all the hooks ******************************/
  for (auto [name, call] : init_hooks) {
    AX_LOG(INFO) << "Running init-hook [" << name << "]";
    AX_CHECK_OK(call()) << "Init-hook [" << name << "] failed.";
  }
  init_hooks.clear();
}

void clean_up() {
  for (auto [name, call] : clean_up_hooks) {
    AX_CHECK_OK(call()) << "CleanUp-hook [" << name << "] failed.";
  }
  registry_p->clear();
  clean_up_hooks.clear();
}

void hook_init(const char* name, std::function<Status()> f) { init_hooks.push_back({name, f}); }

void hook_clean_up(const char* name, std::function<Status()> f) {
  clean_up_hooks.push_back({name, f});
}

entt::registry& global_registry() {
  AX_DCHECK(registry_p != nullptr) << "Registry is not initialized.";
  return *registry_p;
}

entt::dispatcher& global_dispatcher() {
  AX_DCHECK(dispatcher_p != nullptr) << "Registry is not initialized.";
  return *dispatcher_p;
}

}  // namespace ax
