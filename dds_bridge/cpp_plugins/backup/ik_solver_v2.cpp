// IkIpoptSolver.cpp
#include "ik_solver_v2.hpp"

using namespace Ipopt;

IkIpoptSolver::IkIpoptSolver(IKModel &model, const pinocchio::SE3 &target)
    : model_(model), target_(target), lb_(model.getJointLimitsLower()),
      ub_(model.getJointLimitsUpper()) {
    model_.warmStartOptimization(target_);
    // initialize to neutral or last known config
    std::cout << "Current joint configuration: ["
              << model_.getCurrentJointConfig().transpose() * (180 / M_PI)
              << "] deg" << std::endl;
    x_current_ = model_.getCurrentJointConfig();
}

bool IkIpoptSolver::get_nlp_info(Index &n, Index &m, Index &nnz_jac_g,
                                 Index &nnz_h_lag,
                                 IndexStyleEnum &index_style) {
    n = model_.getNumJoints(); // six joints
    m = 0;                     // no constraints
    nnz_jac_g = 0;
    // full symmetric Hessian: n*(n+1)/2
    nnz_h_lag = n * (n + 1) / 2;
    index_style = TNLP::C_STYLE;
    return true;
}

bool IkIpoptSolver::get_bounds_info(Index n, Number *x_l, Number *x_u, Index m,
                                    Number *g_l, Number *g_u) {
    for (Index i = 0; i < n; ++i) {
        x_l[i] = lb_(i);
        x_u[i] = ub_(i);
    }
    // no g
    return true;
}

bool IkIpoptSolver::get_starting_point(Index n, bool init_x, Number *x,
                                       bool init_z, Number * /*z_L*/,
                                       Number * /*z_U*/, Index m,
                                       bool init_lambda, Number * /*lambda*/) {
    if (init_x) {
        for (Index i = 0; i < n; ++i)
            x[i] = x_current_(i);
    }
    return true;
}

bool IkIpoptSolver::eval_f(Index n, const Number *x, bool new_x,
                           Number &obj_value) {
    JointConfig q;
    for (int i = 0; i < 6; ++i)
        q(i) = x[i];
    obj_value = model_.cost(q, target_);
    return true;
}

bool IkIpoptSolver::eval_grad_f(Index n, const Number *x, bool new_x,
                                Number *grad_f) {
    JointConfig q;
    for (int i = 0; i < 6; ++i)
        q(i) = x[i];
    JointConfig g = model_.cost_grad(q, target_);
    for (int i = 0; i < 6; ++i)
        grad_f[i] = g(i);
    return true;
}

bool IkIpoptSolver::eval_g(Index /*n*/, const Number * /*x*/, bool /*new_x*/,
                           Index m, Number * /*g*/) {
    // we have no constraints, so m should be 0 and there’s nothing to fill
    return true;
}

bool IkIpoptSolver::eval_jac_g(Index /*n*/, const Number * /*x*/,
                               bool /*new_x*/, Index m, Index /*nele_jac*/,
                               Index * /*iRow*/, Index * /*jCol*/,
                               Number * /*values*/) {
    // no constraints → nothing to do
    return true;
}
bool IkIpoptSolver::eval_h(Index n, const Number *x, bool new_x,
                           Number obj_factor, Index /*m*/,
                           const Number * /*lambda*/, bool /*new_lambda*/,
                           Index nele_hess, Index *iRow, Index *jCol,
                           Number *values) {
    if (values == nullptr) {
        // return sparsity structure (lower‐triangle)
        Index idx = 0;
        for (Index i = 0; i < n; ++i) {
            for (Index j = 0; j <= i; ++j) {
                iRow[idx] = i;
                jCol[idx] = j;
                ++idx;
            }
        }
    } else {
        // fill in obj_factor * H(i,j)
        JointConfig q;
        for (Index i = 0; i < n; ++i)
            q(i) = x[i];
        auto H = model_.cost_hess(q, target_);
        Index idx = 0;
        for (Index i = 0; i < n; ++i) {
            for (Index j = 0; j <= i; ++j) {
                values[idx++] = obj_factor * H(i, j);
            }
        }
    }
    return true;
}

void IkIpoptSolver::finalize_solution(
    SolverReturn status, Index n, const Number *x, const Number * /*z_L*/,
    const Number * /*z_U*/, Index m, const Number * /*g*/,
    const Number * /*lambda*/, Number /*obj_value*/,
    const IpoptData * /*ip_data*/, IpoptCalculatedQuantities * /*ip_cq*/) {
    // copy back solution
    for (int i = 0; i < 6; ++i)
        x_current_(i) = x[i];
    model_.setCurrentJointConfig(x_current_);
}
