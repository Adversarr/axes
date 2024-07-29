#include "ax/core/init.hpp"

#include <algorithm>
#include <openvdb/openvdb.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <cxxopts.hpp>

#include "ax/core/logging.hpp"
#include "ax/core/entt.hpp"
#include "ax/math/init.hpp"
#include "ax/utils/common.hpp"
#include "ax/utils/time.hpp"

// ABSL_FLAG(int, n_eigen_threads, -1,
//           "Number of eigen parallelism: negative for disable, 0 for hardware cocurrency, positive
//           " "for specific number of threads.");

namespace ax {

/****************************** GLOBAL VARS ******************************/
std::unique_ptr<entt::registry> g_registry;
std::unique_ptr<entt::dispatcher> g_dispatcher;
struct Hook {
  const std::string name_;
  std::function<void()> call_;
};
std::vector<Hook> g_init_hooks;
std::vector<Hook> g_cleanup_hook;

std::string program_path = "UNKNOWN";
std::string logger_name = "ax";
cxxopts::ParseResult parse_result;

const char* get_program_path() { return program_path.c_str(); }

cxxopts::Options& get_program_options() {
  static cxxopts::Options opt("axes", "libaxes internal");
  return opt;
}

cxxopts::ParseResult& get_parse_result() {
  return parse_result;
}

/****************************** Implementation ******************************/

void init(int argc, char** argv) {
  /****************************** Flags ******************************/
  auto& opt = get_program_options();
  opt.add_options()("num_eigen_threads", "Parallelism of Eigen library.",
                    cxxopts::value<int>()->default_value("0"))
                    ("h,help", "Print usage.");
  if (argc > 0) {
    /****************************** Install the debuggers ******************************/
    program_path = argv[0];
    parse_result = opt.parse(argc, argv);
  }

  if (parse_result.count("help")) {
    std::cout << opt.help() << std::endl;
    exit(EXIT_SUCCESS);
  }
  init();
}

std::shared_ptr<spdlog::logger> get_logger() {
  auto logger = spdlog::get(logger_name);
  if (!logger) {
    logger = spdlog::stdout_color_mt(logger_name);
    logger->set_level(spdlog::level::info);
    spdlog::set_default_logger(logger);
    SPDLOG_DEBUG("Initialized spdlog");
  }
  return logger;
}

void init() {
  add_clean_up_hook("Show Timers", []() { erase_resource<utils::TimerRegistry>(); });
  get_logger();  // trigger the logger initialization
  /****************************** Setup Entt Registry ******************************/
  g_registry = std::make_unique<entt::registry>();
  g_dispatcher = std::make_unique<entt::dispatcher>();

  /****************************** Vdb ******************************/
  openvdb::initialize();
  int num_threads = get_parse_result()["num_eigen_threads"].as<int>();
  math::init_parallel(num_threads);
  AX_INFO("Initialized Eigen with {} threads", Eigen::nbThreads());

  // 3. Run all the hooks
  for (auto&& [name, call] : g_init_hooks) {
    AX_INFO("Run init-hook [{}]", name);
    try {
      call();
    } catch (const std::exception& e) {
      AX_CRITICAL("Init-hook [{}] failed: {}", name, e.what());
    }
  }
  g_init_hooks.clear();
}

void clean_up() {
  // 1. Run all the clean up hooks in reverse order.
  std::for_each(g_cleanup_hook.rbegin(), g_cleanup_hook.rend(), [](auto const& hook) {
    AX_INFO("Run clean-up-hook [{}]", hook.name_);
    try {
      hook.call_();
    } catch (const std::exception& e) {
      AX_CRITICAL("Clean up hook [{}] run failed: {}", hook.name_, e.what());
    }
  });
  g_cleanup_hook.clear();

  // 2. Destroy all the resources.
  g_registry.reset();
  g_dispatcher.reset();
  spdlog::drop_all();
}

void add_init_hook(const char* name, std::function<void()> f) {
  for (auto const& [n, _] : g_init_hooks) {
    if (n == name) {
      return;
    }
  }
  g_init_hooks.push_back({name, f});
}

void add_clean_up_hook(const char* name, std::function<void()> f) {
  // Check if the callback is already in the list.
  for (auto const& [n, _] : g_cleanup_hook) {
    if (n == name) {
      return;
    }
  }
  g_cleanup_hook.push_back({name, f});
}

entt::registry& global_registry() {
  AX_DCHECK(g_registry != nullptr, "Registry is not initialized.");
  return *g_registry;
}

entt::dispatcher& global_dispatcher() {
  AX_DCHECK(g_dispatcher != nullptr, "Dispatcher is not initialized.");
  return *g_dispatcher;
}

}  // namespace ax
