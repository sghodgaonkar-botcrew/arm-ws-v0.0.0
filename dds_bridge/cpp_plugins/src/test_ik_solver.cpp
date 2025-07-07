#include "ik_solver.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sys/resource.h>
#include <unistd.h>
#include <vector>

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
        pose_weights << 1.0, 1.0, 1.0, 1.0, 1.0,
            1.0; // Equal weights for all components

        // Create IKModel (assuming URDF is in the urdf directory)
        std::string urdf_path =
            "/home/shanto/Documents/arm-ws-v0.0.0/urdf/arm_v0.1/arm.urdf";
        // std::vector<std::string> possible_paths = {
        //     "./urdf/ur10_v1.1.3/ur10.urdf", "../urdf/ur10_v1.1.3/ur10.urdf",
        //     "../../urdf/ur10_v1.1.3/ur10.urdf",
        //     "../../../urdf/ur10_v1.1.3/ur10.urdf"};

        // bool file_found = false;
        // for (const auto &path : possible_paths) {
        //     std::ifstream file_check(path);
        //     if (file_check.good()) {
        //         urdf_path = path;
        //         file_found = true;
        //         std::cout << "Found URDF file at: " << path << std::endl;
        //         break;
        //     }
        // }

        // if (!file_found) {
        //     std::cerr
        //         << "Error: Could not find URDF file. Tried the following
        //         paths:"
        //         << std::endl;
        //     for (const auto &path : possible_paths) {
        //         std::cerr << "  " << path << std::endl;
        //     }
        //     return 1;
        // }
        auto ik_model = std::make_shared<IKModel>(urdf_path, "connection_frame",
                                                  pose_weights);

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

        // Create IKSolverCeres
        IKSolverCeres ik_solver(ik_model);
        std::cout << "IKSolverCeres created successfully!" << std::endl;

        // Define a target pose (example: slightly offset from current pose)
        pinocchio::SE3 current_pose = ik_model->getEndEffectorFrame();
        pinocchio::SE3 target_pose = current_pose;
        // target_pose.translation().x() = 0.0;
        // target_pose.translation().y() = 0.05;
        target_pose.translation().z() -= 0.2;

        std::cout << "\nTarget pose: [" << target_pose << "]" << std::endl;

        // Measure initial resource usage
        auto start_time = std::chrono::high_resolution_clock::now();
        double cpu_start = getCPUTime();
        long memory_start = getMemoryUsage();

        // Solve IK
        std::cout << "\nSolving inverse kinematics with Ceres..." << std::endl;
        JointConfig solution;
        try {
            solution = ik_solver.solve(target_pose);
            std::cout << "\nOptimization completed!" << std::endl;
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

        std::cout << "Solution joint configuration: [" << solution.transpose()
                  << "]" << std::endl;

        // Verify the solution by computing forward kinematics
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

        // Test with different solver options
        // std::cout << "\n=== Testing with different solver options ==="
        //           << std::endl;
        // ik_solver.setSolverOptions(500, 1e-6, 1e-8);

        // pinocchio::SE3 target_pose2 = current_pose;
        // target_pose2.translation().x() += 0.2;
        // target_pose2.translation().y() += 0.1;

        // std::cout << "New target pose: [" << target_pose2 << "]" <<
        // std::endl;

        // JointConfig solution2 = ik_solver.solve(target_pose2);
        // std::cout << "New solution: [" << solution2.transpose() << "]"
        //           << std::endl;

        // pinocchio::SE3 achieved_pose2 =
        //     ik_model->computeForwardKinematics(solution2);
        // double position_error2 =
        //     (achieved_pose2.translation() -
        //     target_pose2.translation()).norm();
        // std::cout << "New position error: " << position_error2 << " meters"
        //           << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}