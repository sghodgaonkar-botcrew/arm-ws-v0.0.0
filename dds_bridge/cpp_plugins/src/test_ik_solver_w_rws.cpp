#include "ik_solver_w_rws.hpp"
#include "reachable_workspace_generator.h"
#include "robot_model.h"
#include <iostream>

int main(int argc, char *argv[]) {
    try {
        // Initialize robot model
        RobotModel robot_model(
            "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf",
            "connection_frame",
            "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.srdf");
        // Take a system argument (y/n) as to whether or not generate a new
        // reachable workspace
        if (argc > 1) {
            std::string arg1 = argv[1];
            if (arg1 == "y" || arg1 == "Y") {
                std::cout << "Generating new reachable workspace..."
                          << std::endl;
                ReachableWorkspaceGenerator generator(
                    robot_model, // RobotModel reference
                    50000,       // num_points
                    1000,        // batch_size
                    0.05,        // ground_threshold
                    1e-3,        // joint_tolerance
                    10           // max_attempts_per_sample
                );
                generator.generateAndSaveWorkspace("generated_workspaces");
                std::cout << "Reachable workspace generated and saved to "
                             "'generated_workspaces'."
                          << std::endl;
            } else {
                std::cout << "Using existing reachable workspace data."
                          << std::endl;
            }
        } else {
            std::cout << "No argument provided. Using existing reachable "
                         "workspace data."
                      << std::endl;
        }

        // Initialize IK solver with reachable workspace support
        IKSolverWithRWS ik_solver("generated_workspaces", robot_model);

        // Check if workspace data was loaded successfully
        if (!ik_solver.isWorkspaceLoaded()) {
            std::cerr << "Failed to load workspace data!" << std::endl;
            return -1;
        }

        std::cout << "Loaded " << ik_solver.getNumWorkspacePoints()
                  << " workspace points." << std::endl;

        // Example target pose [x, y, z, qx, qy, qz, qw]
        // Define all target poses to test
        std::vector<Eigen::Vector<double, 7>> target_poses = {
            // Original 3 targets
            {0.5, 0.5, 0.5, 0, -0.7071068, 0, 0.7071068},          // Target 1
            {0.5, 0.5, 0.5, 0.5626401, 0, -0.5626401, -0.6056999}, // Target 2
            {0.5, 0.5, 0.5, 0, 0, 0, 1},                           // Target 3

            // New 10 targets with random quaternions
            {-0.117976, 0.657162, 0.856357, 0.123456, 0.234567, 0.345678,
             0.901234}, // Target 4
            {0.259211, 0.114833, 0.148146, -0.234567, 0.345678, 0.456789,
             0.789012}, // Target 5
            {0.710398, 0.109648, 0.173626, 0.345678, -0.456789, 0.567890,
             0.678901}, // Target 6
            {-0.0922635, -0.118152, 0.251824, -0.456789, 0.567890, -0.678901,
             0.567890}, // Target 7
            {-0.325385, -0.350399, 1.096670, 0.567890, 0.678901, 0.789012,
             0.456789}, // Target 8
            {0.106942, 0.934646, 0.214427, -0.678901, 0.789012, 0.890123,
             0.345678}, // Target 9
            {0.0686319, 0.146895, 1.113800, 0.789012, -0.890123, 0.901234,
             0.234567}, // Target 10
            {-0.919380, -0.527017, 0.303057, -0.890123, 0.901234, 0.012345,
             0.123456}, // Target 11
            {-0.110664, -0.109494, 1.052760, 0.901234, 0.012345, -0.123456,
             0.012345}, // Target 12
            {-0.330274, 0.619924, 1.061180, -0.012345, 0.123456, 0.234567,
             0.901234} // Target 13
        };

        // Normalize all quaternions in target poses (keeping [qx, qy, qz, qw]
        // order)
        std::cout << "Normalizing quaternions in target poses..." << std::endl;
        for (auto &pose : target_poses) {
            // Create Eigen quaternion from [qx, qy, qz, qw] components
            // Note: Eigen::Quaterniond constructor expects [w, x, y, z] order
            Eigen::Quaterniond quat(pose[6], pose[3], pose[4],
                                    pose[5]); // [w, x, y, z]

            // Normalize the quaternion using Eigen's built-in function
            quat.normalize();

            // Extract normalized components back to [qx, qy, qz, qw] order
            pose[3] = quat.x(); // qx
            pose[4] = quat.y(); // qy
            pose[5] = quat.z(); // qz
            pose[6] = quat.w(); // qw
        }
        std::cout << "Quaternion normalization completed." << std::endl;

        // Test each target pose
        for (size_t i = 0; i < target_poses.size(); ++i) {
            std::cout << "\n=== Testing Target " << (i + 1)
                      << " ===" << std::endl;
            Eigen::Vector<double, 7> target_pose_xyzquat = target_poses[i];

            std::cout << "\nSolving IK for target pose: ["
                      << target_pose_xyzquat.transpose() << "]" << std::endl;

            // Solve IK
            JointConfig solution = ik_solver.solve(target_pose_xyzquat);

            std::cout << "\nIK solution found!" << std::endl;
            std::cout << "Joint configuration: [" << solution.transpose() << "]"
                      << std::endl;

            // Convert to degrees and wrap if > 180 deg
            Eigen::VectorXd solution_deg = solution * 180.0 / M_PI;
            for (int j = 0; j < solution_deg.size(); ++j) {
                if (solution_deg[j] > 180.0) {
                    solution_deg[j] -= 360.0;
                }
            }
            std::cout << "Joint configuration (degrees): ["
                      << solution_deg.transpose() << "]" << std::endl;

            // Verify the solution by computing forward kinematics
            pinocchio::SE3 final_pose =
                robot_model.computeForwardKinematics(solution);
            XYZQuat final_pose_xyzquat =
                RobotModel::homogeneousToXYZQuat(final_pose);

            std::cout << "\nVerification - Final end-effector pose: ["
                      << final_pose_xyzquat.transpose() << "]" << std::endl;

            // Compute error
            Eigen::Vector<double, 7> error =
                target_pose_xyzquat - final_pose_xyzquat;
            double position_error = error.head<3>().norm();
            double rotation_error = error.tail<4>().norm();

            std::cout << "Position error: " << position_error << " m"
                      << std::endl;
            std::cout << "Rotation error: " << rotation_error << std::endl;
            if (position_error > 1e-6 || rotation_error > 1e-6) {
                std::cout << "WARNING: Target pose is unreachable: error "
                             "exceeds tolerance."
                          << std::endl;

                // Solve IK without warm start (using neutral configuration)
                JointConfig solution_no_warmstart = ik_solver.solve_ik(
                    target_pose_xyzquat, &RobotModel::NEUTRAL_JOINT_CONFIG);

                // Verify the solution without warm start
                pinocchio::SE3 final_pose_no_warmstart =
                    robot_model.computeForwardKinematics(solution_no_warmstart);
                XYZQuat final_pose_xyzquat_no_warmstart =
                    RobotModel::homogeneousToXYZQuat(final_pose_no_warmstart);

                // Compute error for solution without warm start
                Eigen::Vector<double, 7> error_no_warmstart =
                    target_pose_xyzquat - final_pose_xyzquat_no_warmstart;
                double position_error_no_warmstart =
                    error_no_warmstart.head<3>().norm();
                double rotation_error_no_warmstart =
                    error_no_warmstart.tail<4>().norm();

                std::cout << "Solution without warm start:" << std::endl;
                std::cout << "  Final end-effector pose: ["
                          << final_pose_xyzquat_no_warmstart.transpose() << "]"
                          << std::endl;
                std::cout << "  Position error: " << position_error_no_warmstart
                          << " m" << std::endl;
                std::cout << "  Rotation error: " << rotation_error_no_warmstart
                          << std::endl;
                std::cout << "  Joint configuration (degrees): ["
                          << solution_no_warmstart.transpose() * (180.0 / M_PI)
                          << "]" << std::endl;
            } else {
                std::cout << "SUCCESS: Target pose reached within tolerance."
                          << std::endl;
            }
        }

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}