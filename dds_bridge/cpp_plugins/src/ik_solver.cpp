#include "ik_solver.h"
#include <iostream>
#include <stdexcept>

// IKCostFunction implementation
IKCostFunction::IKCostFunction(std::shared_ptr<IKModel> ik_model,
                               const pinocchio::SE3 &target_pose)
    : ik_model_(ik_model), target_pose_(target_pose) {
    if (!ik_model_) {
        throw std::invalid_argument("IKModel pointer cannot be null");
    }
}

bool IKCostFunction::Evaluate(const double *parameters, double *cost,
                              double *gradient) const {
    // Convert parameters to JointConfig
    JointConfig q;
    for (int i = 0; i < 6; ++i) {
        q(i) = parameters[i];
    }

    // Evaluate cost function
    *cost = ik_model_->cost(q, target_pose_);

    // Evaluate gradient if requested
    if (gradient != nullptr) {
        JointConfig grad = ik_model_->cost_grad(q, target_pose_);
        for (int i = 0; i < 6; ++i) {
            gradient[i] = grad(i);
        }
    }

    std::cout << "Cost: " << *cost << std::endl;
    if (gradient != nullptr) {
        std::cout << "Gradient: [";
        for (int i = 0; i < 6; ++i) {
            std::cout << gradient[i];
            if (i < 5)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    return true;
}

// IKSolverCeres implementation
IKSolverCeres::IKSolverCeres(std::shared_ptr<IKModel> ik_model)
    : ik_model_(ik_model) {
    if (!ik_model_) {
        throw std::invalid_argument("IKModel pointer cannot be null");
    }

    // Set default solver options
    setSolverOptions();
}

JointConfig IKSolverCeres::solve(const pinocchio::SE3 &target_pose,
                                 const JointConfig &initial_guess) {
    // Use current joint configuration if no initial guess provided
    JointConfig start_config = initial_guess;
    if (initial_guess.isZero()) {
        start_config = ik_model_->getCurrentJointConfig();
    }

    // Create cost function on heap (Ceres will manage the memory)
    auto cost_function = new IKCostFunction(ik_model_, target_pose);

    // Create gradient problem
    ceres::GradientProblem problem(cost_function);

    // Create solver
    ceres::GradientProblemSolver solver;

    // Solve the problem
    ceres::GradientProblemSolver::Summary summary;
    double parameters[6];
    for (int i = 0; i < 6; ++i) {
        parameters[i] = start_config(i);
    }

    solver.Solve(solver_options_, problem, parameters, &summary);

    // Check if optimization was successful
    if (summary.termination_type != ceres::CONVERGENCE &&
        summary.termination_type != ceres::USER_SUCCESS) {
        std::cerr << "Ceres optimization failed with termination type: "
                  << summary.termination_type << std::endl;
        std::cerr << "Final cost: " << summary.final_cost << std::endl;
        std::cerr << "Number of iterations: " << summary.iterations.size()
                  << std::endl;
    } else {
        std::cout << "Ceres optimization completed successfully!" << std::endl;
        std::cout << "Final cost: " << summary.final_cost << std::endl;
        std::cout << "Number of iterations: " << summary.iterations.size()
                  << std::endl;
        std::cout << "Termination type: " << summary.termination_type
                  << std::endl;
        std::cout << "Function tolerance: "
                  << solver_options_.function_tolerance << std::endl;
        std::cout << "Gradient tolerance: "
                  << solver_options_.gradient_tolerance << std::endl;
        if (!summary.iterations.empty()) {
            const auto &last_iter = summary.iterations.back();
            std::cout << "Last iteration cost change: " << last_iter.cost_change
                      << std::endl;
            std::cout << "Last iteration gradient norm: "
                      << last_iter.gradient_norm << std::endl;
        }
    }

    // Convert solution back to JointConfig
    JointConfig solution;
    for (int i = 0; i < 6; ++i) {
        solution(i) = parameters[i];
    }

    return solution;
}

void IKSolverCeres::setSolverOptions(int max_iterations,
                                     double function_tolerance,
                                     double gradient_tolerance,
                                     double parameter_tolerance) {
    solver_options_.max_num_iterations = max_iterations;
    solver_options_.function_tolerance = function_tolerance;
    solver_options_.gradient_tolerance = gradient_tolerance;
    solver_options_.minimizer_progress_to_stdout =
        true; // Enable progress output
    solver_options_.parameter_tolerance = parameter_tolerance;
}