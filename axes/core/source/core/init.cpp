#include "ax/core/init.hpp"

#include <absl/debugging/failure_signal_handler.h>
#include <absl/debugging/symbolize.h>
#include <absl/flags/parse.h>
#include <absl/log/die_if_null.h>
#include <absl/log/globals.h>
#include <absl/log/initialize.h>
#include <absl/log/log.h>
#include <openvdb/openvdb.h>

#include <Eigen/Core>

#include "ax/core/echo.hpp"
#include "ax/core/entt.hpp"
#include "ax/math/init.hpp"
#include "ax/utils/common.hpp"
#include "ax/utils/status.hpp"
#include "ax/utils/time.hpp"

ABSL_FLAG(int, n_eigen_threads, -1,
          "Number of eigen parallelism: negative for disable, 0 for hardware cocurrency, positive "
          "for specific number of threads.");

namespace ax {

/****************************** GLOBAL VARS ******************************/
UPtr<entt::registry> registry_p;
UPtr<entt::dispatcher> dispatcher_p;

struct Hook {
  const std::string name_;
  std::function<void()> call_;
};

List<Hook> init_hooks;
List<Hook> clean_up_hooks;

const char* program_path = nullptr;

const char* get_program_path() { return program_path; }

/****************************** Implementation ******************************/

void init(int argc, char** argv) {
  if (argc > 0) {
    /****************************** Flags ******************************/
    absl::ParseCommandLine(argc, argv);
    /****************************** Install the debuggers ******************************/
    absl::InitializeSymbolizer(argv[0]);
    program_path = argv[0];
    absl::FailureSignalHandlerOptions failure_signal_handler{};
    absl::InstallFailureSignalHandler(failure_signal_handler);
  } else {
    std::cerr << "Program path is not available: failure signal handler is not installed."
              << std::endl;
  }

  add_clean_up_hook("Show Timers", []() { erase_resource<utils::TimerRegistry>(); });

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
  math::init_parallel(nT);
  AX_LOG(INFO) << "Eigen SIMD instruction sets: " << Eigen::SimdInstructionSetsInUse();

  /****************************** Run all the hooks ******************************/
  for (auto [name, call] : init_hooks) {
    AX_LOG(INFO) << "Run init-hook [" << name << "]";
    try {
      call();
    } catch (const std::exception& e) {
      AX_LOG(FATAL) << "Init-hook [" << name << "] failed: " << e.what();
    }
  }
  init_hooks.clear();
}

void clean_up() {
  for (auto [name, call] : clean_up_hooks) {
    AX_LOG(INFO) << "Run clean-up-hook [" << name << "]";
    try {
      call();
    } catch (const std::exception& e) {
      AX_LOG(FATAL) << "CleanUp-hook [" << name << "] failed: " << e.what();
    }
  }
  clean_up_hooks.clear();
  registry_p.reset();
  dispatcher_p.reset();
}

void add_init_hook(const char* name, std::function<void()> f) { init_hooks.push_back({name, f}); }

void add_clean_up_hook(const char* name, std::function<void()> f) {
  // Check if the callback is already in the list.
  for (auto& [n, _] : clean_up_hooks) {
    if (n == name) {
      return;
    }
  }
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
