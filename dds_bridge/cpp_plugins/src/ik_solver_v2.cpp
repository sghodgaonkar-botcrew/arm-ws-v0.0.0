#include "ik_model.h"
#include <Eigen/Dense>
#include <cstdint>
#include <iostream>
#include <proxsuite/proxqp/dense/dense.hpp>

int main() {
    // Initialize IK model
    IKModel ik_model(
        "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf",
        "connection_frame", Eigen::Vector<double, 6>::Ones());

    const int nx = ik_model.getNumVelocityVariables();
    const int n_eq = 0;
    const int n_in = 0;

    // Neutral start configuration
    Eigen::VectorXd q = IKModel::NEUTRAL_JOINT_CONFIG;

    // Target pose
    pinocchio::SE3 target_pose = pinocchio::SE3::Identity();
    target_pose.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
    target_pose.rotation() =
        Eigen::Quaterniond(0.7071068, 0, -0.7071068, 0).toRotationMatrix();

    // Preallocate
    pinocchio::SE3 fk_pose;
    Eigen::MatrixXd J(6, nx);
    Eigen::Vector<double, 6> err;
    double error_norm = 0.0;

    // Joint velocity bounds
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(nx, -0.1);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(nx, 0.1);

    // QP solver (box constraints = true)
    proxsuite::proxqp::dense::QP<double> qp(nx, n_eq, n_in, true);
    // std::cout << "Box‐constrained? " << std::boolalpha
    //           << qp.is_box_constrained() << std::endl;

    // Line search parameters
    const double alpha_init = 1.0;
    const double rho = 0.5;
    const double alpha_min = 1e-4;

    for (int iter = 0; iter < 1000; ++iter) {
        // 1) Forward kinematics & Jacobian
        fk_pose = ik_model.computeForwardKinematics(q);
        J = ik_model.computeGeometricJacobian(q); // size 6×nx

        // 2) Compute task error
        err = -(pinocchio::log6(target_pose.inverse() * fk_pose)).toVector();
        error_norm = err.norm();
        if (error_norm < 1e-6) {
            std::cout << "Converged in " << iter
                      << " iterations, q = " << q.transpose() * (180 / 3.14159)
                      << std::endl;
            break;
        }
        std::cout << "iter " << iter << "  err = " << error_norm << "\r";

        // 3) Build QP: H = JᵀJ, g = -Jᵀ err
        Eigen::MatrixXd H = J.transpose() * J;
        Eigen::VectorXd g = -J.transpose() * err;

        // 4) Initialize and solve QP with box constraints
        // Note: we supply 9 args: H, g, A_eq, b_eq, C, l, u, lb, ub
        qp.init(H, g, Eigen::MatrixXd(),
                Eigen::VectorXd(), // no equality constraints (A_eq, b_eq)
                Eigen::MatrixXd(),
                Eigen::VectorXd(), // no general inequalities (C, l)
                Eigen::VectorXd(), // no general upper bounds (u)
                lb, ub             // box constraints
        );
        qp.solve();

        Eigen::VectorXd dq = qp.results.x;

        // 5) Backtracking line search along dq
        double alpha = alpha_init;
        const double err0 = error_norm;
        pinocchio::SE3 fk_tmp;
        Eigen::Vector<double, 6> err_tmp;
        double err_tmp_norm;

        while (alpha > alpha_min) {
            Eigen::VectorXd q_test = q + alpha * dq;
            // evaluate new error
            fk_tmp = ik_model.computeForwardKinematics(q_test);
            err_tmp =
                -(pinocchio::log6(target_pose.inverse() * fk_tmp)).toVector();
            err_tmp_norm = err_tmp.norm();
            if (err_tmp_norm < err0)
                break;
            alpha *= rho;
        }

        // 6) Update joint configuration
        q += alpha * dq;
    }

    return 0;
}
