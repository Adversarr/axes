#pragma once

#include "ax/core/buffer/buffer.hpp"
#include "ax/core/buffer/buffer_view.hpp"

namespace ax::fem {

AX_DEFINE_ENUM_CLASS(VariableCondition,
                     None,       ///< No condition, free variable
                     Dirichlet,  ///< Dirichlet boundary condition
                     Neumann     ///< Neumann boundary condition
);

class State {
public:
  State();
  State(size_t n_dof_per_vertex, size_t n_vert, BufferDevice device);
  AX_DECLARE_CONSTRUCTOR(State, default, default);

  BufferPtr<Real> GetVariables() const;

  BufferPtr<VariableCondition> GetCondition() const;

  size_t GetNumDOFPerVertex() const;
  size_t GetNumVertices() const;
  BufferDevice Device() const;


  // Set the data of the state and the condition of each variable.
  void SetData(ConstRealBufferView variables, ConstBufferView<VariableCondition> condition);

private:
  BufferDevice device_;
  BufferPtr<Real> variables_;               ///< variables at each vertex, is (ndof, nvert)
  BufferPtr<VariableCondition> condition_;  ///< condition of each variable, is (ndof, nvert)
  size_t n_dof_per_vertex_ = 0;             ///< number of degrees of freedom per vertex
  size_t n_vert_ = 0;                       ///< number of vertices
};
}  // namespace ax::fem