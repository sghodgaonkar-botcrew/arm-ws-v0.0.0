#include "ik_solver_v2.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sys/resource.h>
#include <unistd.h>
#include <vector>

using namespace Ipopt;

// Helper function to get current memory usage in KB
long getMemoryUsage() {
    FILE *file = fopen("/proc/self/status", "r");
    if (file == nullptr)
        return -1;

    long memory = -1;
    char line[128];
    while (fgets(line, 128, file) != nullptr) {
        if (strncmp(line, "VmRSS:", 6) == 0) {
            sscanf(line, "VmRSS: %ld", &memory);
            break;
        }
    }
    fclose(file);
    return memory;
}

// Helper function to get CPU time in seconds
double getCPUTime() {
    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) == 0) {
        return (double)(usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
               (double)(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) /
                   1000000.0;
    }
    return -1.0;
}

// Helper function to get CPU usage percentage
double getCPUUsagePercentage(double cpu_time, double wall_time) {
    if (wall_time <= 0)
        return 0.0;

    // Get number of CPU cores
    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    if (num_cores <= 0)
        num_cores = 1;

    // Calculate CPU usage percentage
    // CPU usage = (CPU time / Wall time) * 100% / number of cores
    return (cpu_time / wall_time) * 100.0 / num_cores;
}

int main() {
    try {
        // Define pose weights for position and orientation [x, y, z, ωx, ωy,
        // ωz]
        Eigen::Vector<double, 6> pose_weights;
        // pose_weights << 1000.0, 1000.0, 1000.0, 10.0, 10.0,
        //     10.0; // Equal weights for all components

        // Create IKModel (assuming URDF is in the urdf directory)
        // std::string urdf_path =
        //     "/home/shanto/Documents/arm-ws-v0.0.0/urdf/arm_v0.1/arm.urdf";

        std::string urdf_path =
            "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf";
        auto ik_model =
            std::make_shared<IKModel>(urdf_path, "connection_frame");

        std::cout << "IKModel created successfully!" << std::endl;
        std::cout << "Number of joints: " << ik_model->getNumJoints()
                  << std::endl;

        // Print current configuration
        std::cout << "Current joint configuration: ["
                  << ik_model->getCurrentJointConfig().transpose() << "]"
                  << std::endl;

        // Print current end effector pose
        std::cout << "Current end effector pose: ["
                  << ik_model->getEndEffectorFrame() << "]" << std::endl;

        pinocchio::SE3 current_pose = ik_model->getEndEffectorFrame();
        pinocchio::SE3 target_pose = current_pose;
        // target_pose.translation().x() = -0.15;
        // target_pose.translation().y() = 0.1;
        target_pose.translation().z() -= 0.1;

        std::cout << "\nTarget pose: [" << target_pose << "]" << std::endl;

        // Measure initial resource usage
        auto start_time = std::chrono::high_resolution_clock::now();
        double cpu_start = getCPUTime();
        long memory_start = getMemoryUsage();

        // Solve IK
        std::cout << "\nSolving inverse kinematics with IPOPT..." << std::endl;
        JointConfig solution;
        try {
            Ipopt::SmartPtr<Ipopt::TNLP> nlp_ptr =
                new IkIpoptSolver(*ik_model, target_pose);

            Ipopt::SmartPtr<Ipopt::IpoptApplication> app =
                IpoptApplicationFactory();
            app->Options()->SetStringValue("warm_start_init_point", "yes");
            app->Options()->SetIntegerValue("max_iter", 300);
            app->Options()->SetNumericValue("max_cpu_time", 1.0);
            // Relax the final KKT tolerances
            app->Options()->SetNumericValue("tol", 1e-6); // optimality
            app->Options()->SetNumericValue("constr_viol_tol",
                                            1e-6); // feasibility
            app->Options()->SetNumericValue("acceptable_tol",
                                            1e-5); // for acceptable solution
            app->Options()->SetNumericValue("acceptable_constr_viol_tol", 1e-4);
            app->Options()->SetNumericValue("acceptable_compl_inf_tol", 1e-3);
            app->Options()->SetStringValue("mu_strategy",
                                           "monotone"); // or "adaptive"
            // app->Options()->SetNumericValue("barrier_tol",
            //                                 1e-4); // stop barrier loop
            //                                 sooner
            // app->Options()->SetNumericValue("barrier_tol_min",
            // 1e-6); // don’t push beyond this
            app->Options()->SetStringValue("hessian_approximation",
                                           "limited-memory");
            app->Options()->SetIntegerValue("limited_memory_max_history", 20);
            app->Options()->SetStringValue("sb", "yes");
            app->Options()->SetIntegerValue("print_level", 5);
            app->Options()->SetIntegerValue("file_print_level", 5);
            app->Options()->SetStringValue("linear_solver", "mumps");
            app->Options()->SetStringValue("nlp_scaling_method",
                                           "gradient-based");
            app->Options()->SetStringValue("mu_strategy", "adaptive");

            app->Initialize();
            app->OptimizeTNLP(nlp_ptr);
            solution = ik_model->getCurrentJointConfig();
            std::cout << "\nOptimization completed!" << std::endl;
            std::cout << "Solution joint configuration: ["
                      << solution.transpose() * (180 / M_PI) << "] deg "
                      << std::endl;
        } catch (const std::exception &e) {
            std::cout << "\nOptimization failed: " << e.what() << std::endl;
            std::cout << "Using latest available solution..." << std::endl;
            solution = ik_model->getCurrentJointConfig();
        }

        // Measure final resource usage
        auto end_time = std::chrono::high_resolution_clock::now();
        double cpu_end = getCPUTime();
        long memory_end = getMemoryUsage();

        // Calculate and display resource usage
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time);
        double wall_time = duration.count() / 1000000.0; // Convert to seconds
        double cpu_time = cpu_end - cpu_start;
        long memory_used = memory_end - memory_start;

        std::cout << "\n=== Resource Usage ===" << std::endl;
        std::cout << "Wall time: " << wall_time << " seconds" << std::endl;
        std::cout << "CPU time: " << cpu_time << " seconds" << std::endl;
        std::cout << "CPU usage: " << getCPUUsagePercentage(cpu_time, wall_time)
                  << "%" << std::endl;
        std::cout << "Memory usage: " << memory_used << " KB" << std::endl;
        if (memory_start > 0 && memory_end > 0) {
            std::cout << "Peak memory: " << memory_end << " KB" << std::endl;
        }
        pinocchio::SE3 achieved_pose =
            ik_model->computeForwardKinematics(solution);
        XYZQuat achieved_xyzquat = IKModel::homogeneousToXYZQuat(achieved_pose);
        std::cout << "Achieved pose: [" << achieved_xyzquat.transpose() << "]"
                  << std::endl;

        // Compute error
        double position_error =
            (achieved_pose.translation() - target_pose.translation()).norm();
        std::cout << "Position error: " << position_error << " meters"
                  << std::endl;
        // Compute rotation error (in radians)
        Eigen::Matrix3d R_achieved = achieved_pose.rotation();
        Eigen::Matrix3d R_target = target_pose.rotation();
        Eigen::AngleAxisd rot_err(R_achieved.transpose() * R_target);
        double rotation_error = std::abs(rot_err.angle());
        std::cout << "Rotation error: " << rotation_error << " radians"
                  << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}