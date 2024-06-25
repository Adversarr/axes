#include "binding.hpp"

#include <absl/log/globals.h>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <ax/math/common.hpp>
#include <boost/json/stream_parser.hpp>

#include "ax/components/name.hpp"
#include "ax/core/echo.hpp"  // IWYU pragma: export
#include "ax/core/init.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/math/sparse.hpp"

namespace py = pybind11;
using namespace ax;

std::optional<boost::json::value> convert_to_value(py::handle const &src) {
  /* Extract PyObject from handle */
  boost::json::value value;
  PyObject *source = src.ptr();
  // check the actual type of src
  if (PyLong_Check(source)) {
    value = static_cast<idx>(PyLong_AsLong(source));
    return value;
  }
  PyErr_Clear();
  if (PyFloat_Check(source)) {
    value = PyFloat_AsDouble(source);
    return value;
  }
  PyErr_Clear();
  if (PyUnicode_Check(source)) {
    value = std::string(PyUnicode_AsUTF8(source));
    return value;
  }
  PyErr_Clear();
  if (PyBool_Check(source)) {
    value = static_cast<bool>(source == Py_True);
    return value;
  }
  PyErr_Clear();
  return std::nullopt;
}

namespace PYBIND11_NAMESPACE {
namespace detail {
template <> struct type_caster<boost::json::value> {
public:
  /**
   * This macro establishes the name 'inty' in
   * function signatures and declares a local variable
   * 'value' of type inty
   */
  PYBIND11_TYPE_CASTER(boost::json::value, const_name("JsonValue"));

  /**
   * Conversion part 1 (Python->C++): convert a PyObject into a inty
   * instance or return false upon failure. The second argument
   * indicates whether implicit conversions should be applied.
   */
  bool load(handle src, bool) {
    /* Extract PyObject from handle */
    if (auto opt_obj = convert_to_value(src)) {
      value = *opt_obj;
      return true;
    }
    return false;
  }

  /**
   * Conversion part 2 (C++ -> Python): convert an inty instance into
   * a Python object. The second and third arguments are used to
   * indicate the return value policy and parent object (for
   * ``return_value_policy::reference_internal``) and are generally
   * ignored by implicit casters.
   */
  static handle cast(boost::json::value src, return_value_policy /* policy */, handle parent) {
    if (src.is_int64()) {
      return PyLong_FromLong(src.as_int64());
    }
    if (src.is_double()) {
      return PyFloat_FromDouble(src.as_double());
    }
    if (src.is_string()) {
      return PyUnicode_FromString(src.as_string().c_str());
    }
    if (src.is_bool()) {
      return PyBool_FromLong(src.as_bool());
    }
    return parent;
  }
};
}  // namespace detail
}  // namespace PYBIND11_NAMESPACE

namespace axb {

std::vector<char *> &convert_argv(std::vector<std::string> const &argv) {
  static std::vector<char *> argv_c;
  if (argv_c.empty()) {
    for (auto &arg : argv) {
      argv_c.push_back(new char[arg.size() + 1]);
      std::strcpy(argv_c.back(), arg.c_str());
    }
  }
  return argv_c;
}

std::string get_default_float_type() { return "float64"; }

std::string get_default_int_type() { return "int64"; }

static boost::json::object convert_dict_to_json(py::dict const &dict) {
  boost::json::object obj;
  for (auto [k, v] : dict) {
    auto key = k.cast<std::string>();
    auto val = convert_to_value(v);
    if (val) {
      obj[key] = *val;
    } else {
      throw std::runtime_error("Unsupported type in dict: " + key);
    }
  }
  return obj;
}

void bind_core_init(pybind11::module &m) {
  m.def("get_default_float_type", &get_default_float_type)
      .def("get_default_int_type", &get_default_int_type);

  m.def(
       "init",
       [](int argc, std::vector<std::string> argv) { init(argc, convert_argv(argv).data()); },
       py::arg("argc") = 0, py::arg("argv") = std::vector<std::string>())
      .def("clean_up", &clean_up);

  m.def("add_init_hook", &add_init_hook, py::arg("name"), py::arg("f"))
      .def("add_clean_up_hook", &add_clean_up_hook, py::arg("name"), py::arg("f"))
      .def("get_program_path", []() -> std::string { return get_program_path(); });

  /************************* SECT: Entt *************************/
  py::class_<Entity>(m, "Entity")
      .def("__repr__",
           [](Entity const &e) { return std::format("<Entity id={}>", entt::to_integral(e)); })
      .def("__eq__", [](Entity const &e1, Entity const &e2) { return e1 == e2; })
      .def("to_integral", [](Entity const &e) -> idx { return entt::to_integral(e); });

  m.def("create_named_entity",
        [](std::string const &name) -> entt::entity { return cmpt::create_named_entity(name); });

  /************************* SECT: Json *************************/
  m.def("to_json_object", [](std::string const &str) -> boost::json::object {
     boost::json::stream_parser parser;
     parser.write(str.data(), str.size());
     parser.finish();
     return parser.release().as_object();
   }).def("from_json_object", [](boost::json::object const &obj) -> std::string {
    std::ostringstream ss;
    ss << obj;
    return ss.str();
  });

  py::class_<boost::json::object>(m, "JsonObject")
      .def(py::init<>())
      .def(py::init([](std::string const &str) {
        boost::json::stream_parser parser;
        parser.write(str.data(), str.size());
        parser.finish();
        return parser.release().as_object();
      }))
      .def(py::init(&convert_dict_to_json))
      .def("__getitem__",
           [](boost::json::object &obj, std::string const &key) -> boost::json::value & {
             return obj[key];
           })
      .def("__setitem__", [](boost::json::object &obj, std::string const &key,
                             boost::json::value const &val) { obj[key] = val; })
      .def("__setitem__", [](boost::json::object &obj, std::string const &key,
                             std::string const &val) { obj[key] = val; })
      .def("__setitem__",
           [](boost::json::object &obj, std::string const &key, idx val) { obj[key] = val; })
      .def("__setitem__",
           [](boost::json::object &obj, std::string const &key, real val) { obj[key] = val; })
      .def("__setitem__",
           [](boost::json::object &obj, std::string const &key, bool val) { obj[key] = val; })
      .def("__contains__",
           [](boost::json::object &obj, std::string const &key) -> bool {
             return obj.contains(key);
           })
      .def(
          "__iter__",
          [](boost::json::object &obj) { return py::make_iterator(obj.begin(), obj.end()); },
          py::keep_alive<0, 1>())
      .def("__len__", [](boost::json::object &obj) { return obj.size(); })
      .def("__str__",
           [](boost::json::object &obj) {
             std::ostringstream ss;
             ss << obj;
             return ss.str();
           })
      .def("__repr__", [](boost::json::object &obj) {
        std::ostringstream ss;
        ss << obj;
        return ss.str();
      });

  py::class_<boost::json::value>(m, "JsonValue")
      .def(py::init<>())
      .def("__str__",
           [](boost::json::value &val) {
             std::ostringstream ss;
             ss << val;
             return ss.str();
           })
      .def("__repr__",
           [](boost::json::value &val) {
             std::ostringstream ss;
             ss << "<JsonValue kind=" << val.kind() << " value=" << val << ">";
             return ss.str();
           })
      .def("kind", [](boost::json::value &val) { return boost::json::to_string(val.kind()); })
      .def("as_object",
           [](boost::json::value &val) -> boost::json::object { return val.as_object(); })
      .def("as_int", [](boost::json::value &val) -> idx { return val.as_int64(); })
      .def("as_real", [](boost::json::value &val) -> real { return val.as_double(); })
      .def("as_string",
           [](boost::json::value &val) -> std::string { return val.as_string().c_str(); })
      .def("as_bool", [](boost::json::value &val) -> bool { return val.as_bool(); });

  py::class_<boost::json::array>(m, "JsonArray")
      .def(py::init<>())
      .def("__getitem__",
           [](boost::json::array &arr, idx i) -> boost::json::value & { return arr[i]; })
      .def("__setitem__",
           [](boost::json::array &arr, idx i, boost::json::value const &val) { arr[i] = val; })
      .def("__contains__",
           [](boost::json::array &arr, boost::json::value const &val) -> bool {
             return std::find(arr.begin(), arr.end(), val) != arr.end();
           })
      .def(
          "__iter__",
          [](boost::json::array &arr) { return py::make_iterator(arr.begin(), arr.end()); },
          py::keep_alive<0, 1>())
      .def("__len__", [](boost::json::array &arr) { return arr.size(); })
      .def("__str__",
           [](boost::json::array &arr) {
             std::ostringstream ss;
             ss << arr;
             return ss.str();
           })
      .def("__repr__", [](boost::json::array &arr) {
        std::ostringstream ss;
        ss << "<JsonArray size=" << arr.size() << ">";
        return ss.str();
      });

  /************************* SECT: Status *************************/
  py::class_<ax::Status>(m, "Status")
      .def("__str__", [](ax::Status const &status) { return status.ToString(); })
      .def("__repr__", [](ax::Status const &status) { return status.ToString(); });

  /************************* SECT: Cpp Logger *************************/
  m.def("set_log_level", [](const std::string &lvl) {
    if (lvl == "info")
      absl::SetStderrThreshold(absl::LogSeverity::kInfo);
    else if (lvl == "warning")
      absl::SetStderrThreshold(absl::LogSeverity::kWarning);
    else if (lvl == "error")
      absl::SetStderrThreshold(absl::LogSeverity::kError);
    else if (lvl == "fatal")
      absl::SetStderrThreshold(absl::LogSeverity::kFatal);
    else {
      throw std::invalid_argument("Invalid log level: " + lvl);
    }
  });

  /************************* SECT: automatic clean up *************************/
  m.add_object("_clean_up_automatic", py::capsule(&clean_up));
}

void bind_core_math(py::module &m) {
  /************************* SECT: sparse matrix manipulation *************************/
  using namespace ax;
  m.def("make_sparse_matrix",
        static_cast<math::spmatr (*)(idx, idx, math::sp_coeff_list const &)>(
            &math::make_sparse_matrix),
        py::arg("rows"), py::arg("cols"), py::arg("coeff_list"))
      .def(
          "make_sparse_matrix",
          static_cast<math::spmatr (*)(idx, idx, std::vector<idx> const &, std::vector<idx> const &,
                                       std::vector<real> const &)>(&math::make_sparse_matrix),
          py::arg("rows"), py::arg("cols"), py::arg("row"), py::arg("col"), py::arg("val"));

  /************************* SECT: Sparse Solver *************************/
  py::class_<math::SparseSolverBase, UPtr<math::SparseSolverBase>>(m, "SparseSolverBase")
      .def("SetProblem",
           py::overload_cast<SPtr<math::LinsysProblem_Sparse>>(&math::SparseSolverBase::SetProblem))
      .def("SetProblem",
           py::overload_cast<math::spmatr const &>(&math::SparseSolverBase::SetProblem))
      .def("AnalyzePattern", &math::SparseSolverBase::AnalyzePattern)
      .def("Factorize", &math::SparseSolverBase::Factorize)
      .def("Compute", &math::SparseSolverBase::Compute)
      .def("Solve", &math::SparseSolverBase::Solve, py::arg("b"), py::arg("x0") = math::vecxr{})
      .def("SetOptions", &math::SparseSolverBase::SetOptions)
      .def("GetOptions", &math::SparseSolverBase::GetOptions)
      .def("__repr__", [](math::SparseSolverBase const &solver) {
        std::stringstream ss;
        ss << "<SparseSolverBase kind=" << utils::reflect_name(solver.GetKind()).value_or("UNKNOWN")
           << " option=" << solver.GetOptions() << ">";
        return ss.str();
      });

  m.def("get_available_sparse_solvers", []() -> std::vector<std::string> {
    return utils::reflect_names<math::SparseSolverKind>();
  });

  py::class_<math::LinsysSolveResult>(m, "LinsysSolveResult")
      .def_readonly("num_iter", &math::LinsysSolveResult::num_iter_)
      .def_readonly("l2_err", &math::LinsysSolveResult::l2_err_)
      .def_readonly("linf_err", &math::LinsysSolveResult::linf_err_)
      .def_readonly("converged", &math::LinsysSolveResult::converged_)
      .def_readonly("solution", &math::LinsysSolveResult::solution_);

  m.def("make_sparse_solver", [](std::string solver_name) -> UPtr<math::SparseSolverBase> {
    auto solver_kind = utils::reflect_enum<math::SparseSolverKind>(solver_name);
    if (!solver_kind) {
      throw std::runtime_error("Invalid solver name: " + solver_name);
    }
    return math::SparseSolverBase::Create(*solver_kind);
  });
}

void bind_core_module(py::module &m) {
  bind_core_init(m);
  bind_core_math(m);
}

}  // namespace axb
