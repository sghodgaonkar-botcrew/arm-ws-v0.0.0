#include "ik_model.h"
#include "ik_solver_v2.hpp"
#include <chrono>
#include <iostream>
#include <vector>

// Enable profiling for this example
#define ENABLE_PROFILING

int main() {
    std::cout << "=== Simple IPOPT Solver Profiling ===" << std::endl;

    try {
        // Reset profile data
        IKModel::resetProfileData();
        IkIpoptSolver::resetProfileData();

        // Load the robot model
        std::string urdf_path =
            "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf";
        std::string end_effector_name = "connection_frame";
        Eigen::Vector<double, 6> pose_weights =
            Eigen::Vector<double, 6>::Ones();

        std::cout << "Loading IK model..." << std::endl;
        IKModel ik_model(urdf_path, end_effector_name, pose_weights);

        std::cout << "\nTesting IK operations performance..." << std::endl;

        // Test multiple random targets
        for (int test = 0; test < 10; ++test) {
            std::cout << "\n--- Test " << (test + 1) << "/10 ---" << std::endl;

            // Generate random target pose
            pinocchio::SE3 target_pose = pinocchio::SE3::Random();
            std::cout << "Target pose: " << target_pose << std::endl;

            // Test forward kinematics
            auto start = std::chrono::high_resolution_clock::now();
            pinocchio::SE3 fk_result = ik_model.computeForwardKinematics(
                ik_model.getCurrentJointConfig());
            auto end = std::chrono::high_resolution_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                      start);
            std::cout << "  Forward kinematics: " << duration.count() << " μs"
                      << std::endl;

            // Test cost function
            start = std::chrono::high_resolution_clock::now();
            double cost_value =
                ik_model.cost(ik_model.getCurrentJointConfig(), target_pose);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(
                end - start);
            std::cout << "  Cost function: " << duration.count()
                      << " μs (value: " << cost_value << ")" << std::endl;

            // Test gradient computation
            start = std::chrono::high_resolution_clock::now();
            JointConfig gradient = ik_model.cost_grad(
                ik_model.getCurrentJointConfig(), target_pose);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(
                end - start);
            std::cout << "  Gradient computation: " << duration.count() << " μs"
                      << std::endl;

            // Test Hessian computation
            start = std::chrono::high_resolution_clock::now();
            Eigen::Matrix<double, 6, 6> hessian = ik_model.cost_hess(
                ik_model.getCurrentJointConfig(), target_pose);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(
                end - start);
            std::cout << "  Hessian computation: " << duration.count() << " μs"
                      << std::endl;

            // Test warm start optimization
            start = std::chrono::high_resolution_clock::now();
            ik_model.warmStartOptimization(target_pose, 50, 1e-3);
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(
                end - start);
            std::cout << "  Warm start optimization: " << duration.count()
                      << " μs" << std::endl;

            // Test solver creation (without running optimization)
            try {
                start = std::chrono::high_resolution_clock::now();
                IkIpoptSolver solver(ik_model, target_pose);
                end = std::chrono::high_resolution_clock::now();
                duration =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        end - start);
                std::cout << "  Solver creation: " << duration.count() << " μs"
                          << std::endl;
            } catch (const std::exception &e) {
                std::cout << "  Solver creation failed: " << e.what()
                          << std::endl;
            }
        }

        // Print comprehensive profile reports
        std::cout << "\n" << std::string(80, '=') << std::endl;
        std::cout << "COMPREHENSIVE PERFORMANCE ANALYSIS" << std::endl;

        // Print IK model profile report
        try {
            IKModel::printProfileReport();
        } catch (const std::exception &e) {
            std::cout << "Error printing IK model profile: " << e.what()
                      << std::endl;
        }

        // Print IK solver profile report
        try {
            IkIpoptSolver::printProfileReport();
        } catch (const std::exception &e) {
            std::cout << "Error printing IK solver profile: " << e.what()
                      << std::endl;
        }

        // Performance analysis and recommendations
        std::cout << "\n=== PERFORMANCE ANALYSIS ===" << std::endl;
        std::cout << "Based on the profiling results:" << std::endl;
        std::cout << "1. Forward kinematics is typically the fastest operation"
                  << std::endl;
        std::cout << "2. Cost function includes FK + error computation"
                  << std::endl;
        std::cout
            << "3. Gradient computation is more expensive (includes Jacobian)"
            << std::endl;
        std::cout << "4. Hessian computation is the most expensive (includes "
                     "multiple Jacobians)"
                  << std::endl;
        std::cout << "5. Warm start optimization provides good initial guesses"
                  << std::endl;
        std::cout << "6. IPOPT solver creation overhead should be minimized"
                  << std::endl;

        std::cout << "\n=== OPTIMIZATION RECOMMENDATIONS ===" << std::endl;
        std::cout << "1. Cache forward kinematics results when possible"
                  << std::endl;
        std::cout << "2. Reuse solver instances instead of creating new ones"
                  << std::endl;
        std::cout << "3. Use warm starts to reduce IPOPT iterations"
                  << std::endl;
        std::cout << "4. Consider approximating Hessian for speed" << std::endl;
        std::cout << "5. Profile the actual IPOPT optimization separately"
                  << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}