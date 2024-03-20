/**
 * @file axes/graph/graph.hpp
 * @brief Graph class
 *
 */

#pragma once
#include "axes/core/entt.hpp"
#include "axes/utils/common.hpp"
#include "axes/utils/meta.hpp"

namespace ax::graph {

  class Graph;
class GraphExecutorBase;


AX_DECLARE_ENUM(PinIOType){
    kInput = 0,   // Input pin
    kOutput = 1,  // Output pin
};

AX_DECLARE_ENUM(PinStatus){
    kConnected = 0,     // Pin is connected
    kDisconnected = 1,  // Pin is disconnected
};

/************************* SECT: DataLoad *************************/
template <typename PayloadType> class DataLoad;
/**
 * @brief DataLoadBase class: The base class of DataLoad.
 */
class DataLoadBase {
public:
  bool HasChanged() const noexcept { return changed_; }
  
  void ResetChanged() noexcept { changed_ = false; }

protected:
  explicit DataLoadBase(void* data);

  virtual ~DataLoadBase() = default;

  template<typename Payload = void>
  Payload* GetPayload() noexcept { return static_cast<Payload*>(data_); }

  void WritePayload(void* data) noexcept {
    data_ = data;
    changed_ = true;
  }


  template <typename PayloadType> DataLoad<PayloadType>* Cast(PayloadType* = nullptr) noexcept {
    return static_cast<DataLoad<PayloadType>*>(this);
  }

  template <typename PayloadType> DataLoad<PayloadType> const* Cast(PayloadType* = nullptr) const noexcept {
    return static_cast<DataLoad<PayloadType> const*>(this);
  }

private:
  void* data_;
  bool changed_;
};

/**
 * @brief DataLoad class: Perform static polymorphism to store the data.
 */
template <typename PayloadType> class DataLoad : public DataLoadBase {
public:
  DataLoad() : data_(std::make_unique<PayloadType>()), DataLoadBase(data_.get()) {}

  template<typename ... Args>
  explicit DataLoad(Args&& ... args) : data_(std::make_unique<PayloadType>(std::forward<Args>(args)...)) {}

  virtual ~DataLoad() = default;

  AX_DECLARE_CONSTRUCTOR(DataLoad, delete, delete);

  PayloadType* GetPayload() noexcept { return static_cast<PayloadType*>(GetPayload()); }

  void WritePayload(PayloadType const& data) {
    *data_ = data;
    DataLoadBase::WritePayload(data_.get());
  }

  void WritePayload(PayloadType&& data) {
    *data_ = std::move(data);
    DataLoadBase::WritePayload(data_.get());
  }

  template <typename ... Args>
  void EmplacePayload(Args&& ... args) {
    data_ = std::make_unique<PayloadType>(std::forward<Args>(args)...);
    DataLoadBase::WritePayload(data_.get());
  }

  void ReplacePayload(std::unique_ptr<PayloadType> data) {
    data_ = std::move(data);
    DataLoadBase::WritePayload(data_.get());
  }

private:
  std::unique_ptr<PayloadType> data_;
};

/************************* SECT: Pin Information *************************/

/**
 * @brief PinInfo class
 *
 * This class is used to store the information of a pin.
 * It contains the type of the pin, the name, description, default value of the pin.
 * The PinInfo does not store the actual data of the pin, it only stores the information of the pin.
 */
class PinDescriptor {
public:
  explicit PinDescriptor(PinIOType io_type, std::type_index tindex, std::string const& name,
                   std::string const& description = "", void* default_value=nullptr);

  std::type_index GetType() const;
  std::string GetName() const;
  std::string GetDescription() const;
  PinIOType GetIOType() const;
  bool GetDefaultValue(void* data_write_buffer);

private:
  std::type_index type_;
  std::string name_;
  std::string description_;
  PinIOType io_type_;
  void* default_value_;
};

template <typename T>
PinDescriptor make_input_pin_descriptor(std::string const& name, std::string const& description = "", void* default_value) {
  return PinDescriptor(PinIOType::kInput, typeid(T), name, description, default_value);
}

template <typename T>
PinDescriptor make_output_pin_descriptor(std::string const& name,
                                         std::string const& description = "") {
  return PinDescriptor(PinIOType::kOutput, typeid(T), name, description);
}

/************************* SECT: Pin IO *************************/

/**
 * @brief PinIO class This class is used to store the data of a pin.
 *
 * It contains the information of the pin, and one optional POINTER to the actual data of the pin.
 * The PinIO does not store the information of the pin, it only stores the data of the pin.
 */
class PinIO {
public:
  AX_DECLARE_CONSTRUCTOR(PinIO, delete, delete);
  /************************* SECT: Connectivity *************************/
  bool IsConnected();

  bool CheckConnect(PinIO const& another);

  void Connect(PinIO const& another);

  void Disconnect();

  /************************* SECT: Data *************************/
  void AttachData(DataLoadBase* dataload);
  DataLoadBase* GetData();

private:
  DataLoadBase* dataload_;
};


/************************* SECT: Node *************************/

AX_DECLARE_ENUM(NodeRunStrategy){
    kSequential,  // Run node sequentially, will guide the execution to run immediately after its
                  // input is ready.
    kParallel     // Run node in parallel,  will guide the execution to run in parallel with other
                  // nodes.
};

AX_DECLARE_ENUM(NodeTriggerStrategy){
    kManual,  // Trigger node manually
    kLazy,    // Trigger node only once, and everytime its input changed.
    kActive   // Trigger node every frame
};

class NodeDescriptorFactoryBase;

class NodeBase {
public:
  virtual Status Run() = 0;

  virtual Status Setup();

  virtual Status CleanUp();

  std::vector<DataLoadBase*> const& GetInputData();

  std::vector<DataLoadBase*> const& GetOutputData();

  void SetInputSize(size_t size) { input_data_.resize(size); }
  void SetOutputSize(size_t size) { output_data_.resize(size); }

//protected:
  template <typename... Args> std::tuple<Args const&...> RetriveInput() {
    return RetriveInputImpl(std::make_index_sequence<sizeof...(Args)>);
  }

  template <typename... Args, size_t... i> auto RetriveInputImpl(std::index_sequence<i...>) {
    return std::tuple<Args const&...>{input_data_[i]->template GetPayload<Args>()};
  }

  template <typename... Args> auto RetriveOutput() {
    return RetriveOutputImpl<Args...>(std::make_index_sequence<sizeof...(Args)>());
  }

  template <typename... Args, size_t... i> auto RetriveOutputImpl(std::index_sequence<i...>) {
    return std::make_tuple(static_cast<Args const&>(*output_data_[i]->Cast(&args).GetPayload());
  }

private:
  std::vector<DataLoadBase*> input_data_;
  std::vector<DataLoadBase*> output_data_;
};


class NodeDescriptor {
public:
  friend class NodeDescriptorFactoryBase;
  std::unique_ptr<NodeBase> Create() const { return creator_(); }
  std::vector<PinDescriptor> const& GetInputPins() const;
  std::vector<PinDescriptor> const& GetOutputPins() const;

  NodeRunStrategy GetRunStrategy() const { return run_strategy; }
  NodeTriggerStrategy GetTriggerStrategy() const { return trigger_strategy; }
  std::string const& GetName() const { return name_; }

private:
  NodeDescriptor(std::string name, std::type_index node_type,
                 std::function<std::unique_ptr<NodeBase>()> creator, NodeRunStrategy run_strategy,
                 NodeTriggerStrategy trigger_strategy)
      : node_type_(node_type),
        name_(name),
        run_strategy(run_strategy),
        trigger_strategy(trigger_strategy),
        creator_(creator) {}
  void AddInputPin(PinDescriptor const& pin) { input_pins_.push_back(pin);}
  void AddOutputPin(PinDescriptor const& pin) { output_pins_.push_back(pin); }
  std::vector<PinDescriptor> input_pins_;
  std::vector<PinDescriptor> output_pins_;
  std::type_index node_type_;
  std::string name_;
  const NodeRunStrategy run_strategy;
  const NodeTriggerStrategy trigger_strategy;
  std::function<std::unique_ptr<NodeBase>()> creator_;
};

class NodeDescriptorFactoryBase {
public:
  AX_DECLARE_CONSTRUCTOR(NodeDescriptorFactoryBase, delete, delete);

  NodeDescriptorFactoryBase& In(PinDescriptor const& pin) {
    descriptor_->AddInputPin(pin);
    return *this;
  }
  NodeDescriptorFactoryBase& Out(PinDescriptor const& pin) {
    descriptor_->AddOutputPin(pin); return *this;
  }

  template <typename T> NodeDescriptorFactoryBase& In(std::string const& name,
                                                      std::string const& description = "",
                                                      void* default_value = nullptr) {
    return In(make_input_pin_descriptor<T>(name, description, default_value));
  }

  template <typename T>
  NodeDescriptorFactoryBase& Out(std::string const& name, std::string const& description = "") {
    return Out(make_output_pin_descriptor<T>(name, description));
  }

  std::unique_ptr<NodeDescriptor> Build() { return std::move(descriptor_); }
 
protected:
  NodeDescriptorFactoryBase(std::string name, std::type_index node_type,
                            std::function<std::unique_ptr<NodeBase>()> creator,
                            NodeRunStrategy run_strategy, NodeTriggerStrategy trigger_strategy)
      : descriptor_(new NodeDescriptor(name, node_type, creator, run_strategy, trigger_strategy)) {}

  std::unique_ptr<NodeDescriptor> descriptor_;
};

template <typename Node> class NodeDescriptorFactory : public NodeDescriptorFactoryBase {
  static_assert(std::is_base_of_v<NodeBase, Node>, "Node must be derived from NodeBase");

public:
  NodeDescriptorFactory(std::string name, NodeRunStrategy run_strategy, NodeTriggerStrategy trigger_strategy)
      : NodeDescriptorFactoryBase(
          name, typeid(Node), []() { return std::make_unique<Node>(); }, run_strategy,
          trigger_strategy) {}
};

}  // namespace ax::graph
