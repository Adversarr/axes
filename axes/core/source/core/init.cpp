#include "ax/core/init.hpp"

#include <absl/debugging/failure_signal_handler.h>
#include <absl/debugging/symbolize.h>
#include <absl/flags/parse.h>
#include <absl/log/die_if_null.h>
#include <absl/log/globals.h>
#include <absl/log/initialize.h>
#include <absl/log/log.h>
#include <openvdb/openvdb.h>

#include "ax/core/echo.hpp"
#include "ax/core/entt.hpp"
#include "ax/math/init.hpp"
#include "ax/utils/common.hpp"
#include "ax/utils/status.hpp"
#include "ax/utils/time.hpp"

ABSL_FLAG(int, n_eigen_threads, 0, "Number of eigen parallelism");

namespace ax {

/****************************** GLOBAL VARS ******************************/
UPtr<entt::registry> registry_p;
UPtr<entt::dispatcher> dispatcher_p;

struct Hook {
  const char* name_;
  std::function<Status()> call_;
};

List<Hook> init_hooks;
List<Hook> clean_up_hooks;

const char* program_path = nullptr;

const char* get_program_path() { return program_path; }

/****************************** Implementation ******************************/

void init(int argc, char** argv) {
  using namespace absl;
  /****************************** Flags ******************************/
  absl::ParseCommandLine(argc, argv);

  /****************************** Install the debuggers ******************************/
  AX_CHECK(argc > 0) << "argc must be greater than 0";
  absl::InitializeSymbolizer(argv[0]);
  program_path = argv[0];
  FailureSignalHandlerOptions failure_signal_handler{};
  absl::InstallFailureSignalHandler(failure_signal_handler);
  add_clean_up_hook("Show Timers", []() {
    erase_resource<utils::TimerRegistry>();
    AX_RETURN_OK();
  });

  init();
}

void init() {
  absl::InitializeLog();
  // absl::SetMinLogLevel(absl::LogSeverityAtLeast::kInfo);
  // absl::SetStderrThreshold(absl::LogSeverity::kInfo);
  /****************************** Setup Entt Registry ******************************/
  registry_p = std::make_unique<entt::registry>();
  dispatcher_p = std::make_unique<entt::dispatcher>();

  /****************************** Vdb ******************************/
  openvdb::initialize();
  int nT = absl::GetFlag(FLAGS_n_eigen_threads);
  if (nT > 1) {
    math::init_parallel();
  }

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

void add_init_hook(const char* name, std::function<Status()> f) { init_hooks.push_back({name, f}); }

void add_clean_up_hook(const char* name, std::function<Status()> f) {
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
