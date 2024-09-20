#include "ax/fem/utils/prune_dbc.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/buffer/for_each.hpp"
#include "ax/utils/cuda_helper.hpp"
#include "prune_dbc_gpu.hpp"
#include <gsl/assert>
namespace ax::fem {

void do_prune_host(RealBufferView grad, ConstBufferView<VariableCondition> bc, ConstRealBufferView bc_var) {
  for_each(std::tuple{grad, bc, bc_var}, [](Real& value, VariableCondition const& bc, Real const& var) {
    if (bc == VariableCondition::Dirichlet) {
      value = var;
    }
  });
}

void do_prune_hessian_host(math::RealBlockMatrix& hessian, ConstBufferView<VariableCondition> bc) {
  auto rp = hessian.RowPtrs()->ConstView();
  auto ci = hessian.ColIndices()->ConstView();
  auto val = hessian.Values()->View();
  size_t bs = hessian.BlockSize();

  for_each_indexed(Dim1{rp.Shape().X() - 1}, [=](size_t i) mutable {
    int begin = rp(i), end = rp(i + 1);
    for (int bid = begin; bid < end; ++bid) {
      // block i, j.
      int j = ci(static_cast<size_t>(bid));
      size_t ii = i;
      size_t jj = static_cast<size_t>(j);
      for (size_t l = 0; l < bs; ++l) {
        for (size_t k = 0; k < bs; ++k) {
          // row i * bs + k, col j * bs + l.
          auto bc_ik = bc(k, ii);
          auto bc_jl = bc(l, jj);
          if (bc_ik == VariableCondition::Dirichlet || bc_jl == VariableCondition::Dirichlet) {
            if (ii == jj && k == l) {
              val(k, l, static_cast<size_t>(bid)) = 1;
            } else {
              val(k, l, static_cast<size_t>(bid)) = 0;
            }
          }
        }
      }
    }
  });
}

void prepare_prune_host(ConstBufferView<VariableCondition> bc, RealBufferView variables, ConstRealBufferView var_in) {
  auto [x, y, _1] = *variables.Shape();
  par_for_each_indexed(Dim2{x, y}, [&](size_t i, size_t j) {
    if (bc(i, j) == VariableCondition::Dirichlet) {
      variables(i, j) = var_in(i, j);
    } else {
      variables(i, j) = 0;
    }
  });
}

void PruneDirichletBc::Prune(RealBufferView grad) {
  auto bc = state_->GetCondition();
  if (!bc) {
    return;
  }

  auto device = bc->Device();
  if (grad.Device() != device) {
    AX_THROW_RUNTIME_ERROR("input is not on the same device as the boundary condition");
  }

  if (device == BufferDevice::Host) {
    do_prune_host(grad, bc->ConstView(), bc_var_->ConstView());
  } else {
    AX_CUDA_CALL(do_prune_gpu(grad, bc->ConstView(), bc_var_->ConstView()));
  }
}

void PruneDirichletBc::Prune(math::RealBlockMatrix& hessian) {
  auto bc = state_->GetCondition();
  if (!bc) {
    return;
  }

  auto device = bc->Device();
  if (hessian.Device() != device) {
    AX_THROW_RUNTIME_ERROR("input is not on the same device as the boundary condition");
  }

  if (device == BufferDevice::Host) {
    do_prune_hessian_host(hessian, bc->ConstView());
  } else {
    AX_CUDA_CALL(do_prune_hessian_gpu(hessian, bc->ConstView()));
  }
}

PruneDirichletBc::PruneDirichletBc(std::shared_ptr<State> state) : state_(std::move(state)) {}

void PruneDirichletBc::PruneWithGradient(math::RealBlockMatrix& hessian, RealBufferView grad) {
  // NOTE: It seems that, if you are using the correct `grad`, this should be enough, we do not need to
  //       update the `grad` as follows.
  // hessian.Multiply(bc_var_->ConstView(), grad, -1, 1); // grad = grad - hessian * bc_var
  Prune(hessian);
  Prune(grad);
}

void PruneDirichletBc::UpdateDbcValue() {
  auto bc = state_->GetCondition();
  auto var = state_->GetVariables();
  if (!bc) {
    return;
  }

  auto device = bc->Device();
  bc_var_ = ensure_buffer<Real>(bc_var_, device, var->Shape());

  if (device == BufferDevice::Host) {
    // copy the Dirichlet value to variables_, other values are zero.
    prepare_prune_host(bc->ConstView(), bc_var_->View(), var->ConstView());
  } else {
    AX_CUDA_CALL(prepare_prune_gpu(bc->ConstView(), bc_var_->View(), var->ConstView()));
  }
}

}  // namespace ax::fem