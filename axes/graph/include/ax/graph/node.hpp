#pragma once
#include "ax/core/common.hpp"
#include "ax/core/config.hpp"
#include "ax/core/echo.hpp"
#include "ax/core/status.hpp"
#include "ax/graph/common.hpp"
#include "ax/graph/payload.hpp"
#include "ax/graph/pin.hpp"

#include <vector>
#include <string>

namespace ax::graph {
using NodeConstructor = std::function<UPtr<NodeBase>(NodeDescriptor const*, idx)>;

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

  NodeDescriptorFactory& AddInput(PinDescriptor const& desc) {
    descriptor_.inputs_.push_back(desc);
    return *this;
  }

  NodeDescriptorFactory& AddOutput(PinDescriptor const& desc) {
    descriptor_.outputs_.push_back(desc);
    return *this;
  }

  NodeDescriptor Finalize() { return descriptor_; }

private:
  NodeDescriptor descriptor_;
};

class NodeBase {
public:
  virtual ~NodeBase() = default;
  // Core apply function, will be called by the graph system.
  virtual Status Apply(idx frame_id) = 0;
  virtual Status PreApply(idx frame_id);
  virtual Status PostApply(idx frame_id);

  // Call from Create().
  virtual Status OnConstruct();
  virtual Status OnDestroy();

  // Some node can have temporary data, such as a RenderMesh node, will register an 
  // entity to the scene, and use the internal renderer to render it.
  virtual Status CleanUp();

  // Getters:
  idx GetId() const { return id_; }
  std::string GetName() const { return descriptor_->name_; }
  std::string GetDescription() const { return descriptor_->description_; }
  TypeIdentifier GetType() const { return descriptor_->type_; }

  Pin const* GetInput(idx index) const { return &inputs_[index]; }
  Pin const* GetOutput(idx index) const { return &outputs_[index]; }

  idx GetNumInputs() const { return inputs_.size(); }
  idx GetNumOutputs() const { return outputs_.size(); }

  std::vector<Pin> const& GetInputs() const { return inputs_; }
  std::vector<Pin> const& GetOutputs() const { return outputs_; }

protected:
  // TODO: io.
  void* RetriveInput(idx index, std::type_index check_type);

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
};

namespace details {


NodeDescriptor const * factory_register(NodeDescriptor desc);

NodeDescriptor const *get_node_descriptor(std::string name);

std::vector<std::string> const& get_node_names();

} // namespace details


}