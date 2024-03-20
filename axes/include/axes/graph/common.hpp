/**
 * @file axes/graph/graph.hpp
 * @brief Graph class
 *
 */

#pragma once
#include <typeindex>

#include "axes/core/entt.hpp"

namespace ax::graph {

/**
 * @brief The Identifier for Type.
 *
 */
struct TypeID {
  std::type_index index_;
  std::string name_;
};

enum class PinIOType : int {
  kInput,  // Input pin
  kOutput  // Output pin
};

enum class PinStatus {
  kConnected,    // Pin is connected
  kDisconnected  // Pin is disconnected
};

class PinInfo {
public:
  TypeID const& GetType() const;
  std::string GetName() const;
  std::string GetDescription() const;
  PinIOType GetIOType() const;
  bool GetDefaultValue(void* data_write_buffer);
  size_t DataTypeSize() const;
};

class PinIO {
public:
  /************************* SECT: Meta *************************/
  PinInfo const& GetInfo() const;

  /************************* SECT: Connectivity *************************/
  bool IsConnected();

  bool CheckConnect(PinIO const& another);

  void Connect(PinIO const& another);

  void Disconnect();

  /************************* SECT: Data *************************/

};

AX_DECLARE_ENUM(NodeRunStrategy){
    kSequential,  // Run node sequentially
    kParallel     // Run node in parallel
};

AX_DECLARE_ENUM(NodeTriggerStrategy){
    kManual,  // Trigger node manually
    kAuto,    // Trigger node only once, and everytime its input changed.
    kLoop     // Trigger node every frame
};

class NodeBase {
public:
  NodeBase(NodeRunStrategy run_strategy, NodeTriggerStrategy trigger_strategy);

  virtual Status operator()() = 0;
  virtual std::vector<PinInfo> const& GetInputPinsInfo() const = 0;
  virtual std::vector<PinInfo> const& GetOutputPinsInfo() const = 0;

  std::vector<void*> const& GetInputPins();
  const NodeRunStrategy run_strategy;
  const NodeTriggerStrategy trigger_strategy;
};

}  // namespace ax::graph
