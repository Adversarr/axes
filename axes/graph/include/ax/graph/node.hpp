#pragma once
#include "ax/core/common.hpp"
#include "ax/core/config.hpp"
#include "ax/core/status.hpp"
#include "ax/graph/common.hpp"
#include "ax/graph/payload.hpp"
#include "ax/graph/pin.hpp"

#include <boost/json/object.hpp>
#include <vector>
#include <string>

namespace ax::graph {
using NodeConstructor = std::function<UPtr<NodeBase>(NodeDescriptor const*, idx)>;

namespace details {

NodeDescriptor const * factory_register(NodeDescriptor desc);

NodeDescriptor const *get_node_descriptor(const char* name);

std::vector<std::string> const& get_node_names();

} // namespace details


struct NodeDescriptor {
  NodeDescriptor(TypeIdentifier type): type_(type) {}

  TypeIdentifier type_;
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

template <typename T>
class NodeDescriptorFactory {
public:
  static_assert(std::is_base_of_v<NodeBase, T>, "T must be derived from NodeBase");
  NodeDescriptorFactory() : descriptor_(typeid(T)) {
    descriptor_.ctor_ = [](NodeDescriptor const* descript, idx id) -> UPtr<NodeBase> {
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

  template<typename Tp>
  NodeDescriptorFactory& AddInput(std::string name,
                                  std::string description) {
    details::ensure_ctor_dtor<Tp>();
    descriptor_.inputs_.push_back(PinDescriptor{typeid(Tp), name, description});
    return *this;
  }


  template<typename Tp>
  NodeDescriptorFactory& AddOutput(std::string name,
                                   std::string description) {
    details::ensure_ctor_dtor<Tp>();
    descriptor_.outputs_.push_back(PinDescriptor{typeid(Tp), name, description});
    return *this;
  }

  NodeDescriptor Finalize() { 
    return descriptor_;
  }

  void FinalizeAndRegister() {
    details::factory_register(descriptor_);
  }

private:
  NodeDescriptor descriptor_;
};

class NodeBase {
public:
  virtual ~NodeBase() = default;
  // Core apply function, will be called by the graph system.
  virtual Status Apply(idx frame_id);
  virtual Status PreApply();
  virtual Status PostApply();

  // Reserved for future use
  virtual Status OnConnect(idx in_io_index);


  // Call from Create().
  virtual Status OnConstruct();
  virtual void OnDestroy();

  // Some node can have temporary data, such as a RenderMesh node, will register an 
  // entity to the scene, and use the internal renderer to render it.
  virtual void CleanUp();

  // Serialize and Deserialize
  virtual boost::json::object Serialize() const;
  virtual void Deserialize(boost::json::object const& obj);

  // Getters:
  idx GetId() const { return id_; }
  NodeDescriptor const* GetDescriptor() const { return descriptor_; }
  TypeIdentifier GetType() const { return descriptor_->type_; }

  Pin const* GetInput(idx index) const { return &inputs_[index]; }
  Pin const* GetOutput(idx index) const { return &outputs_[index]; }

  idx GetNumInputs() const { return inputs_.size(); }
  idx GetNumOutputs() const { return outputs_.size(); }

  std::vector<Pin> const& GetInputs() const { return inputs_; }
  std::vector<Pin> const& GetOutputs() const { return outputs_; }

protected:
  template <typename T, typename ... Args>
  T* SetOutput(idx index, Args && ... arg) {
    T* p = RetriveOutput<T>(index);
    if (p != nullptr) {
      *p = T(std::forward<Args>(arg)...);
    }
    return p;
  }

  template<typename T>
  T* RetriveInput(idx index) {
    if ((idx) inputs_.size() <= index || index < 0) {
      return nullptr;
    }
    if (Payload* p = inputs_[index].payload_; p != nullptr) {
      return p->Cast<T>();
    } else {
      return nullptr;
    }
  }

  template<typename T>
  T* RetriveOutput(idx index) {
    if ((idx) outputs_.size() <= index || index < 0) {
      return nullptr;
    }
    if (Payload* p = outputs_[index].payload_; p != nullptr) {
      return p->Cast<T>();
    } else {
      return nullptr;
    }
  }

  template<typename T>
  T const* RetriveOutput(idx index) const {
    if ((idx) outputs_.size() <= index || index < 0) {
      return nullptr;
    }
    if (Payload* p = outputs_[index].payload_; p != nullptr) {
      return p->Cast<T>();
    } else {
      return nullptr;
    }
  }

  Pin* GetInput(idx index) { return &inputs_[index]; }

  Pin* GetOutput(idx index) { return &outputs_[index]; }

  NodeBase(NodeDescriptor const* descriptor, idx id);

private:
  // Whatever, although friend is not recommended, but it is the only way to make the
  // factory function to be able to access the constructor, and user cannot access it.
  friend class Graph;
  // Factory
  static UPtr<NodeBase> Create(NodeDescriptor const* descript, idx id);


  NodeDescriptor const* descriptor_;
  idx const id_;
  // Node should not care about the memory allocation of input params.
  std::vector<Pin> inputs_;

  // Node should take care of the memory allocation of output params.
  std::vector<Pin> outputs_;
  std::vector<Payload> output_payloads_;
};

}
