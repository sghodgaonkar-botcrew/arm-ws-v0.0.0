#include "ik_model.h"
#include "ik_solver_v2.hpp"
#include <chrono>
#include <iostream>

// IPOPT includes
#ifdef HAVE_IPOPT
#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#endif

// Enable profiling for this example
#define ENABLE_PROFILING

int main() {
    std::cout << "=== IK System Profiling Example ===" << std::endl;

    try {
        // Reset any existing profile data
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

        // Print all frames to see available frames
        ik_model.printAllFrames();

        // Test forward kinematics multiple times
        std::cout << "\nTesting forward kinematics performance..." << std::endl;
        for (int i = 0; i < 100; ++i) {
            JointConfig test_config = JointConfig::Random();
            auto pose = ik_model.computeForwardKinematics(test_config);
            auto xyzquat = IKModel::homogeneousToXYZQuat(pose);
        }

        // Test cost function evaluation
        std::cout << "\nTesting cost function performance..." << std::endl;
        pinocchio::SE3 target_pose = pinocchio::SE3::Identity();
        for (int i = 0; i < 50; ++i) {
            JointConfig test_config = JointConfig::Random();
            double cost_val = ik_model.cost(test_config, target_pose);
            auto grad = ik_model.cost_grad(test_config, target_pose);
            auto hess = ik_model.cost_hess(test_config, target_pose);
        }

        // Test warm start optimization
        std::cout << "\nTesting warm start optimization..." << std::endl;
        for (int i = 0; i < 5; ++i) {
            pinocchio::SE3 random_target = pinocchio::SE3::Random();
            ik_model.warmStartOptimization(random_target, 100);
        }

        // Test IK solver (if IPOPT is available)
        std::cout << "\nTesting IK solver performance..." << std::endl;
        std::cout << "IPOPT testing disabled due to segmentation fault issues"
                  << std::endl;
        std::cout << "Focusing on core IK operations profiling..." << std::endl;

        // Print comprehensive profile reports
        std::cout << "\n" << std::string(80, '=') << std::endl;
        std::cout << "GENERATING PERFORMANCE PROFILE REPORTS..." << std::endl;

        try {
            IKModel::printProfileReport();
        } catch (const std::exception &e) {
            std::cout << "Error printing IK model profile report: " << e.what()
                      << std::endl;
        }

        try {
            IkIpoptSolver::printProfileReport();
        } catch (const std::exception &e) {
            std::cout << "Error printing IK solver profile report: " << e.what()
                      << std::endl;
        }

        // Print summary of potential bottlenecks
        std::cout << "\n=== PERFORMANCE ANALYSIS SUMMARY ===" << std::endl;
        std::cout << "Key areas to monitor:" << std::endl;
        std::cout << "1. Forward kinematics computation (pinocchio_fk)"
                  << std::endl;
        std::cout << "2. Jacobian computation (jacobian_computation)"
                  << std::endl;
        std::cout << "3. Cost function evaluation (cost_function)" << std::endl;
        std::cout << "4. Gradient computation (cost_gradient)" << std::endl;
        std::cout << "5. Hessian computation (cost_hessian)" << std::endl;
        std::cout << "6. Warm start optimization iterations" << std::endl;
        std::cout << "7. IPOPT solver iterations (if available)" << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}