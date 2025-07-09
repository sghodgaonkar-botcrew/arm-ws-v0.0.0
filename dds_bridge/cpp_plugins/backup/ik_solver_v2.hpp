// IkIpoptSolver.h
#pragma once

#include "ik_model.h"
#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>

using namespace Ipopt;

class IkIpoptSolver : public TNLP {
  public:
    /// Construct with an IKModel, target SE3, and joint limits (size 6)
    IkIpoptSolver(IKModel &model, const pinocchio::SE3 &target);

    virtual ~IkIpoptSolver() = default;

    // IPOPT methods:
    // No constraints
    bool get_nlp_info(Index &n, Index &m, Index &nnz_jac_g, Index &nnz_h_lag,
                      IndexStyleEnum &index_style) override;

    bool get_bounds_info(Index n, Number *x_l, Number *x_u, Index m,
                         Number *g_l, Number *g_u) override;

    bool get_starting_point(Index n, bool init_x, Number *x, bool init_z,
                            Number *z_L, Number *z_U, Index m, bool init_lambda,
                            Number *lambda) override;

    bool eval_f(Index n, const Number *x, bool new_x,
                Number &obj_value) override;

    bool eval_grad_f(Index n, const Number *x, bool new_x,
                     Number *grad_f) override;

    // — Correct eval_g signature —
    bool eval_g(Index n, const Number *x, bool new_x, Index m,
                Number *g) override;

    // — Correct eval_jac_g signature —
    bool eval_jac_g(Index n, const Number *x, bool new_x, Index m,
                    Index nele_jac, Index *iRow, Index *jCol,
                    Number *values) override;

    // — Correct eval_h signature —
    // in class IkIpoptSolver : public Ipopt::TNLP
    bool eval_h(Index n, const Number *x, bool new_x, Number obj_factor,
                Index m, const Number *lambda,
                bool new_lambda, // <— insert this
                Index nele_hess, Index *iRow, Index *jCol,
                Number *values) override;

    void finalize_solution(SolverReturn status, Index n, const Number *x,
                           const Number *z_L, const Number *z_U, Index m,
                           const Number *g, const Number *lambda,
                           Number obj_value, const IpoptData *ip_data,
                           IpoptCalculatedQuantities *ip_cq) override;
    // …

  private:
    IKModel &model_;
    pinocchio::SE3 target_;
    Eigen::VectorXd lb_, ub_;
    JointConfig x_current_;
};
