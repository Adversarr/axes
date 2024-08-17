#include "ax/core/init.hpp"

#include <openvdb/openvdb.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#ifdef DO_PRAGMA
#undef DO_PRAGMA
#endif

#if defined(_MSC_VER)
    #define DISABLE_WARNING_PUSH           __pragma(warning( push ))
    #define DISABLE_WARNING_POP            __pragma(warning( pop ))
    #define DISABLE_WARNING(warningNumber) __pragma(warning( disable : warningNumber ))

#elif defined(__GNUC__) || defined(__clang__)
    #define DO_PRAGMA(X) _Pragma(#X)
    #define DISABLE_WARNING_PUSH           DO_PRAGMA(GCC diagnostic push)
    #define DISABLE_WARNING_POP            DO_PRAGMA(GCC diagnostic pop)
    #define DISABLE_WARNING(warningName)   DO_PRAGMA(GCC diagnostic ignored #warningName)

#else
    #define DISABLE_WARNING_PUSH
    #define DISABLE_WARNING_POP
#endif
#include <Eigen/Core>
#include <algorithm>
#include <cxxopts.hpp>

// backward: [-Wzero-as-null-pointer-constant] [-Wold-style-cast] should disable
#ifdef AX_HAS_LIBDW
#  define BACKWARD_HAS_DW 1
#endif
DISABLE_WARNING_PUSH
#ifdef _MSC_VER
DISABLE_WARNING(4265)
DISABLE_WARNING(5031)
#else
DISABLE_WARNING(-Wzero-as-null-pointer-constant)
DISABLE_WARNING(-Wold-style-cast)
#endif
#include "3rdparty/backward/backward.hpp"
DISABLE_WARNING_POP

#include "ax/core/entt.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/init_parallel.hpp"
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

const char* get_program_path() { return kProgramPath.c_str(); }

cxxopts::Options& get_program_options() {
  static cxxopts::Options opt("axes", "libaxes internal argument parser");
  return opt;
}

cxxopts::ParseResult& get_parse_result() {
  return kParseResult;
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
    kProgramPath = argv[0];
    kParseResult = opt.parse(argc, argv);
  }

  if (kParseResult.count("help")) {
    std::cout << opt.help() << std::endl;
    exit(EXIT_SUCCESS);
  }
  init();
}

std::shared_ptr<spdlog::logger> get_logger() {
  auto logger = spdlog::get(kLoggerName);
  if (!logger) {
    logger = spdlog::stdout_color_mt(kLoggerName);
    logger->set_level(spdlog::level::info);
    logger->set_pattern("[%D %T.%e] [%^%L%$] %s:%#: %v");
    spdlog::set_default_logger(logger);
    AX_DEBUG("Initialized spdlog");
  }
  return logger;
}

void init() {
  static backward::SignalHandling sh; // install the signal handler.

  add_clean_up_hook("Show Timers", []() { erase_resource<utils::TimerRegistry>(); });
  get_logger();  // trigger the logger initialization
  /****************************** Setup Entt Registry ******************************/
  kRegistry = std::make_unique<entt::registry>();
  kDispatcher = std::make_unique<entt::dispatcher>();

  /****************************** Vdb ******************************/
  openvdb::initialize();
  int num_threads = get_parse_result()["num_eigen_threads"].as<int>();
  math::init_parallel(num_threads);
  AX_INFO("Initialized Eigen with {} threads", Eigen::nbThreads());

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
