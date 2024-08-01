#pragma once
#include <boost/json/object.hpp>
#include <boost/preprocessor/facilities/va_opt.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/for_each_i.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/tuple/push_back.hpp>
#include <boost/preprocessor/variadic/size.hpp>
#include <boost/preprocessor/variadic/to_seq.hpp>
#include <string>
#include <vector>

#include "ax/core/common.hpp"
#include "ax/core/config.hpp"
#include "ax/graph/common.hpp"
#include "ax/graph/payload.hpp"
#include "ax/graph/pin.hpp"

namespace ax::graph {
using NodeConstructor = std::function<std::unique_ptr<NodeBase>(NodeDescriptor const*, size_t)>;

namespace details {

NodeDescriptor const* factory_register(NodeDescriptor desc);

NodeDescriptor const* get_node_descriptor(const char* name);

std::vector<std::string> const& get_node_names();

}  // namespace details

struct NodeDescriptor {
  explicit NodeDescriptor(type_index type) : type_(type) {}

  type_index type_;
  std::string name_;
  std::string description_;
  std::vector<PinDescriptor> inputs_;
  std::vector<PinDescriptor> outputs_;

protected:
  friend class Graph;
  friend class NodeBase;
  template <typename T> friend class NodeDescriptorFactory;
  NodeConstructor ctor_;
};

template <typename T> class NodeDescriptorFactory {
public:
  static_assert(std::is_base_of_v<NodeBase, T>, "T must be derived from NodeBase");
  NodeDescriptorFactory() : descriptor_(typeid(T)) {
    descriptor_.ctor_ = [](NodeDescriptor const* descript, size_t id) -> std::unique_ptr<NodeBase> {
      return std::make_unique<T>(descript, id);
    };
  }

  NodeDescriptorFactory& SetName(std::string name) {
    descriptor_.name_ = std::move(name);
    return *this;
  }

  NodeDescriptorFactory& SetDescription(std::string description) {
    descriptor_.description_ = std::move(description);
    return *this;
  }

  template <typename Tp>
  NodeDescriptorFactory& AddInput(std::string name, std::string description) {
    details::ensure_ctor_dtor<Tp>();
    descriptor_.inputs_.push_back(PinDescriptor{typeid(Tp), name, description});
    return *this;
  }

  template <typename Tp>
  NodeDescriptorFactory& AddOutput(std::string name, std::string description) {
    details::ensure_ctor_dtor<Tp>();
    descriptor_.outputs_.push_back(PinDescriptor{typeid(Tp), name, description});
    return *this;
  }

  NodeDescriptor Finalize() { return descriptor_; }

  void FinalizeAndRegister() { details::factory_register(descriptor_); }

private:
  NodeDescriptor descriptor_;
};

class NodeBase {
public:
  virtual ~NodeBase() = default;
  // Core apply function, will be called by the graph system.
  virtual void Apply(size_t frame_id);
  virtual void PreApply();
  virtual void PostApply();

  // Reserved for future use
  virtual void OnConnect(size_t in_io_index);

  // Some node can have temporary data, such as a RenderMesh node, will register an
  // entity to the scene, and use the internal renderer to render it.
  virtual void CleanUp() noexcept;

  // Serialize and Deserialize
  virtual boost::json::object Serialize() const;
  virtual void Deserialize(boost::json::object const& obj);

  // Getters:
  size_t GetId() const { return id_; }
  NodeDescriptor const* GetDescriptor() const { return descriptor_; }
  type_index GetType() const { return descriptor_->type_; }

  Pin const* GetInput(size_t index) const { return &inputs_[index]; }
  Pin const* GetOutput(size_t index) const { return &outputs_[index]; }

  size_t GetNumInputs() const { return inputs_.size(); }
  size_t GetNumOutputs() const { return outputs_.size(); }

  std::vector<Pin> const& GetInputs() const { return inputs_; }
  std::vector<Pin> const& GetOutputs() const { return outputs_; }

protected:
  template <typename T, typename... Args> T* SetOutput(size_t index, Args&&... arg) {
    T* p = RetriveOutput<T>(index);
    if (p != nullptr) {
      *p = T(std::forward<Args>(arg)...);
    }
    return p;
  }

  template <typename T> T* RetriveInput(size_t index) {
    if (inputs_.size() <= index) {
      return nullptr;
    }
    if (Payload* p = inputs_[index].payload_; p != nullptr) {
      return p->Cast<T>();
    }
    return nullptr;
  }

  template <typename T> T* RetriveOutput(size_t index) {
    if (outputs_.size() <= index || index < 0) {
      return nullptr;
    }
    if (Payload* p = outputs_[index].payload_; p != nullptr) {
      return p->Cast<T>();
    }
    return nullptr;
  }

  template <typename T> T const* RetriveOutput(size_t index) const {
    if (outputs_.size() <= index || index < 0) {
      return nullptr;
    }
    if (Payload* p = outputs_[index].payload_; p != nullptr) {
      return p->Cast<T>();
    }
    return nullptr;
  }

  Pin* GetInput(size_t index) { return &inputs_[index]; }

  Pin* GetOutput(size_t index) { return &outputs_[index]; }

  NodeBase(NodeDescriptor const* descriptor, size_t id);

private:
  // Whatever, although friend is not recommended, but it is the only way to make the
  // factory function to be able to access the constructor, and user cannot access it.
  friend class Graph;
  // Factory
  static std::unique_ptr<NodeBase> Create(NodeDescriptor const* descript, ident_t id);

  NodeDescriptor const* descriptor_;
  ident_t const id_;
  // Node should not care about the memory allocation of input params.
  std::vector<Pin> inputs_;

  // Node should take care of the memory allocation of output params.
  std::vector<Pin> outputs_;
  std::vector<Payload> output_payloads_;
};

}  // namespace ax::graph

/************************* SECT: Helper Macros *************************/
#define AX_DECLARE_INPUT(T, name, desc, ith)         \
  struct in_##name##_t {                             \
    using type = T;                                  \
    static constexpr size_t index = ith;             \
    static constexpr char const* description = desc; \
  };                                                 \
  template <> struct input_param_i<ith> {            \
    using type = in_##name##_t;                      \
  };                                                 \
  static constexpr in_##name##_t in_##name = {};     \
  const T* Get(in_##name##_t) { return RetriveInput<T>(ith); }

#define AX_DECLARE_OUTPUT(T, name, desc, ith)                                        \
  struct out_##name##_t {                                                            \
    using type = T;                                                                  \
    static constexpr size_t index = ith;                                             \
    static constexpr char const* description = desc;                                 \
  };                                                                                 \
  static constexpr out_##name##_t out_##name = {};                                   \
  template <> struct output_param_i<ith> {                                           \
    using type = out_##name##_t;                                                     \
  };                                                                                 \
  template <typename... Args> T* Set(out_##name##_t, Args&&... value) { /* NOLINT */ \
    return SetOutput<T>(ith, std::forward<Args>(value)...);                          \
  }

#define AX_DECLARE_INPUT_ADAPTOR(r, data, ith, elem) \
  AX_DECLARE_INPUT BOOST_PP_TUPLE_PUSH_BACK(elem, ith);
#define AX_DECLARE_OUTPUT_ADAPTOR(r, data, ith, elem) \
  AX_DECLARE_OUTPUT BOOST_PP_TUPLE_PUSH_BACK(elem, ith);

// DO NOT USE THIS
#define AX_NODE_ADD_INPUT(r, data, elem)                                                          \
  factory.AddInput<BOOST_PP_TUPLE_ELEM(0, elem)>(BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(1, elem)), \
                                                                    BOOST_PP_TUPLE_ELEM(2, elem));

// DO NOT USE THIS
#define AX_NODE_ADD_OUTPUT(r, data, elem)          \
  factory.AddOutput<BOOST_PP_TUPLE_ELEM(0, elem)>( \
      BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(1, elem)), BOOST_PP_TUPLE_ELEM(2, elem));

// Declare the input of one node. e.g. AX_NODE_INPUTS((type, name, description), ...)
#define AX_NODE_OUTPUTS(...)                                                            \
  BOOST_PP_VA_OPT((BOOST_PP_SEQ_FOR_EACH_I(AX_DECLARE_OUTPUT_ADAPTOR, _,                \
                                           BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))),     \
                  (), __VA_ARGS__)                                                      \
  static void register_outputs(NodeDescriptorFactory<ThisType>& factory) {              \
    BOOST_PP_SEQ_FOR_EACH(AX_NODE_ADD_OUTPUT, _, BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__)) \
  }

// Declare the output of one node. e.g. AX_NODE_OUTPUTS((type, name, description), ...)
#define AX_NODE_INPUTS(...)                                                                  \
  BOOST_PP_VA_OPT((BOOST_PP_SEQ_FOR_EACH_I(AX_DECLARE_INPUT_ADAPTOR, _,                      \
                                           BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))),          \
                  (), __VA_ARGS__)                                                           \
  static void register_inputs(NodeDescriptorFactory<ThisType>& factory) {                    \
    BOOST_PP_SEQ_FOR_EACH(AX_NODE_ADD_INPUT, _, BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))       \
  }                                                                                          \
  template <size_t... Idx> inline auto GetAllInputImpl(std::index_sequence<Idx...>) {        \
    return std::make_tuple(Get(input_param_i_t<Idx>{})...);                                  \
  }                                                                                          \
  inline auto GetAllInput() {                                                                \
    return GetAllInputImpl(std::make_index_sequence<BOOST_PP_VARIADIC_SIZE(__VA_ARGS__)>{}); \
  }

#define AX_NODE_DETAILS_CALL(r, data, elem) elem();

// Declare the common part of the node.
#define AX_NODE_COMMON(name, title, desc, ...)                                                   \
  using ThisType = name;                                                                         \
  static constexpr char const* title_ = title;                                                   \
  static constexpr char const* desc_ = desc;                                                     \
  static void register_input(NodeDescriptorFactory<name>& factory);                              \
  static void register_output(NodeDescriptorFactory<name>& factory);                             \
  static void register_this() {                                                                  \
    NodeDescriptorFactory<name> f;                                                               \
    f.SetName(title_).SetDescription(desc_);                                                     \
    register_input(f);                                                                           \
    register_output(f);                                                                          \
    f.FinalizeAndRegister();                                                                     \
    BOOST_PP_VA_OPT(                                                                             \
        (BOOST_PP_SEQ_FOR_EACH(AX_NODE_DETAILS_CALL, _, BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))), \
        (void(0));                                                                               \
        , __VA_ARGS__)                                                                           \
  }                                                                                              \
  template <size_t i> struct input_param_i {};                                                   \
  template <size_t i> using input_param_i_t = typename input_param_i<i>::type;                   \
  template <size_t i> struct output_param_i {};                                                  \
  template <size_t i> using output_param_i_t = typename output_param_i<i>::type;                 \
  template <typename... Args> auto Get(Args const&... args) {                                    \
    return std::make_tuple(Get(args)...);                                                        \
  }                                                                                              \
  name(NodeDescriptor const* descriptor, ident_t id) : NodeBase(descriptor, id)

// Declare the common part of the node with constructor.
#define AX_NODE_COMMON_WITH_CTOR(name, title, desc, ...) \
  AX_NODE_COMMON(name, title, desc __VA_OPT__(, ) __VA_ARGS__) {}

// Helper to ensure the input is not nullptr.
#define AX_NODE_INPUT_ENSURE(name)                                          \
  (([this]() -> typename in_##name##_t::type const* {                       \
    auto const* ptr = this->Get(in_##name);                                 \
    if (ptr == nullptr) {                                                   \
      throw std::invalid_argument("Input \"" #name "\" is not connected."); \
    }                                                                       \
    return ptr;                                                             \
  })())

#define AX_NODE_INPUT_ENSURE_EXTRACT(name) (*AX_NODE_INPUT_ENSURE(name))

#define AX_NODE_INPUT_EXTRACT_DEFAULT(name, default_value) \
  (([this]() -> typename in_##name##_t::type {             \
    auto const* ptr = this->Get(in_##name);                \
    if (ptr == nullptr) {                                  \
      return default_value;                                \
    }                                                      \
    return *ptr;                                           \
  })())
