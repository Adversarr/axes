#include "ax/core/init.hpp"

#include <openvdb/openvdb.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <Eigen/Core>
#include <algorithm>
#include <cxxopts.hpp>

// backward: [-Wzero-as-null-pointer-constant] [-Wold-style-cast] should disable
#ifdef AX_HAS_LIBDW
#  define BACKWARD_HAS_DW 1
#endif
CXXOPTS_DIAGNOSTIC_PUSH
CXXOPTS_IGNORE_WARNING("-Wzero-as-null-pointer-constant")
CXXOPTS_IGNORE_WARNING("-Wold-style-cast")
#include "3rdparty/backward/backward.hpp"
CXXOPTS_DIAGNOSTIC_POP

#include "ax/core/entt.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/utils/init_parallel.hpp"
#include "ax/utils/time.hpp"

namespace ax {

/****************************** GLOBAL VARS ******************************/
std::unique_ptr<entt::registry> kRegistry;
std::unique_ptr<entt::dispatcher> kDispatcher;

struct Hook {
  const std::string name_;
  std::function<void()> call_;
};

std::vector<Hook> kInitHooks;
std::vector<Hook> kCleanupHooks;

std::string kProgramPath;
std::string kLoggerName = "ax";
cxxopts::ParseResult kParseResult;

const char* get_program_path() {
  return kProgramPath.c_str();
}

/****************************** Implementation ******************************/

void initialize(int argc, char** argv) {
  /****************************** Flags ******************************/
  auto& opt = po::get_program_options();
  opt.add_options()("num_eigen_threads", "Parallelism of Eigen library.",
                    cxxopts::value<int>()->default_value("0"))(
      "log_level", "Log level", cxxopts::value<std::string>()->default_value("info"))(
      "h,help", "Print usage.");

  if (argc > 0) {
    /****************************** Install the debuggers ******************************/
    kProgramPath = argv[0];
    kParseResult = opt.parse(argc, argv);
  }

  if (kParseResult.count("help")) {
    std::cout << opt.help() << std::endl;
    exit(EXIT_SUCCESS);
  }
  initialize();
}

std::shared_ptr<spdlog::logger> get_logger() {
  auto logger = spdlog::get(kLoggerName);
  if (!logger) {
    logger = spdlog::stdout_color_mt(kLoggerName);
    logger->set_level(spdlog::level::info);
    logger->set_pattern("[%D %T.%e] [%^%L%$] %s:%#: %v");
    set_default_logger(logger);
    AX_DEBUG("Initialized spdlog");
  }
  return logger;
}

void initialize() {
  static backward::SignalHandling sh;  // install the signal handler.

  add_clean_up_hook("Show Timers", []() {
    erase_resource<utils::TimerRegistry>();
  });
  get_logger();  // trigger the logger initialization
  /****************************** Setup Entt Registry ******************************/
  kRegistry = std::make_unique<entt::registry>();
  kDispatcher = std::make_unique<entt::dispatcher>();

  /****************************** Vdb ******************************/
  openvdb::initialize();

  std::string log_level = po::get_parse_result()["log_level"].as<std::string>();
  if (log_level.front() == 'i') {
    spdlog::default_logger()->set_level(spdlog::level::info);
  } else if (log_level.front() == 'd') {
    spdlog::default_logger()->set_level(spdlog::level::debug);
  } else if (log_level.front() == 't') {
    spdlog::default_logger()->set_level(spdlog::level::trace);
  } else if (log_level.front() == 'w') {
    spdlog::default_logger()->set_level(spdlog::level::warn);
  } else if (log_level.front() == 'e') {
    spdlog::default_logger()->set_level(spdlog::level::err);
  } else if (log_level.front() == 'c') {
    spdlog::default_logger()->set_level(spdlog::level::critical);
  } else if (log_level.front() == 'o') {
    spdlog::default_logger()->set_level(spdlog::level::off);
  } else {
    AX_CRITICAL("Unknown log level: {}", log_level);
    abort();
  }

  if (const int num_threads = po::get_parse_result()["num_eigen_threads"].as<int>();
      num_threads != 1) {
    math::init_parallel(num_threads);
    AX_INFO("Initialized Eigen with {} threads", Eigen::nbThreads());
  }

  // 3. Run all the hooks
  for (auto&& [name, call] : kInitHooks) {
    AX_INFO("Run init: {}", name);
    try {
      call();
    } catch (const std::exception& e) {
      AX_CRITICAL("Init-hook [{}] failed: {}", name, e.what());
      abort();
    }
  }
  kInitHooks.clear();
}

namespace po {

Options& get_program_options() {
  static Options opt("axes", "libaxes internal argument parser");
  return opt;
}

ParseResult& get_parse_result() {
  return kParseResult;
}

void add_option(const Option& option) {
  auto& opt = get_program_options();
  opt.add_option("", option);
}

void add_option(std::initializer_list<Option> opt_list) {
  for (auto const& opt : opt_list) {
    add_option(opt);
  }
}

}  // namespace po

void clean_up() {
  // 1. Run all the clean up hooks in reverse order.
  std::for_each(kCleanupHooks.rbegin(), kCleanupHooks.rend(), [](auto const& hook) {
    AX_INFO("Run clean up: {}", hook.name_);
    try {
      hook.call_();
    } catch (const std::exception& e) {
      AX_CRITICAL("Clean up hook [{}] run failed: {}", hook.name_, e.what());
    }
  });
  kCleanupHooks.clear();

  // 2. Destroy all the resources.
  openvdb::uninitialize();
  kDispatcher.reset();
  kRegistry.reset();
  spdlog::drop_all();
}

void add_init_hook(const char* name, std::function<void()> f) {
  for (auto const& [n, _] : kInitHooks) {
    if (n == name) {
      return;
    }
  }
  kInitHooks.push_back({name, f});
}

void add_clean_up_hook(const char* name, std::function<void()> f) {
  // Check if the callback is already in the list.
  for (auto const& [n, _] : kCleanupHooks) {
    if (n == name) {
      return;
    }
  }
  kCleanupHooks.push_back({name, f});
}

entt::registry& global_registry() {
  AX_DCHECK(kRegistry != nullptr, "Registry is not initialized.");
  return *kRegistry;
}

entt::dispatcher& global_dispatcher() {
  AX_DCHECK(kDispatcher != nullptr, "Dispatcher is not initialized.");
  return *kDispatcher;
}

void print_stack() {
  backward::StackTrace st;
  st.load_here(32);
  backward::Printer p;
  p.print(st);
}

}  // namespace ax
