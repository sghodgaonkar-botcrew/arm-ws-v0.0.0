#include "arm_msgs.hpp"
#include "ik_solver_w_rws.hpp"
#include "motor_controller.h"
#include "robot_model.h"
#include "workspace_types.h"
#include <algorithm>
#include <chrono>
#include <dds/dds.hpp>
#include <iomanip>
#include <iostream>
#include <thread>

/**
 * @brief Returns the current time in seconds with nanosecond precision.
 *
 * @return double Time since epoch in seconds.
 */
double now_seconds() {
    using namespace std::chrono;
    auto t = high_resolution_clock::now().time_since_epoch();
    return duration<double>(t).count();
}

/// @brief Macro to log INFO messages with a timestamp prefix and a fixed
/// precision.
#define LOG_INFO()                                                             \
    std::cout << "[INFO] [" << std::fixed << std::setprecision(9)              \
              << now_seconds() << "] [IK SUB] "

/// @brief Placeholder acceleration limit value for joint movement
#define ACCEL_LIMIT NaN

/**
 * @brief Main entry point for the IK subscriber executable.
 *
 * Subscribes to a DDS topic `ArmCommandTopic`, receives pose commands,
 * computes joint angles (placeholder logic here), and sends commands to a motor
 * controller.
 */
int main(int argc, char **argv) {
    // Initialize DDS participant and topic
    dds::domain::DomainParticipant participant(0);
    dds::topic::Topic<arm_msgs::ArmCommand> topic(participant,
                                                  "ArmCommandTopic");

    // Create DDS subscriber with QoS policies
    auto subscriber = dds::sub::Subscriber(participant);
    auto dr_qos = subscriber.default_datareader_qos()
                  << dds::core::policy::Reliability::Reliable()
                  // << dds::core::policy::History::KeepLast(10)
                  << dds::core::policy::Durability::Volatile();
    dds::sub::DataReader<arm_msgs::ArmCommand> reader(subscriber, topic,
                                                      dr_qos);

    // Instantiate the motor controller
    MotorController controller;

    // Initialize robot model and IK solver
    RobotModel robot_model(
        "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf",
        "connection_frame",
        "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.srdf");

    // Initialize IK solver with reachable workspace support
    IKSolverWithRWS *ik_solver_ptr = nullptr;
    bool workspace_available = false;

    try {
        ik_solver_ptr =
            new IKSolverWithRWS("generated_workspaces", robot_model);
        workspace_available = ik_solver_ptr->isWorkspaceLoaded();

        if (workspace_available) {
            LOG_INFO() << "Loaded " << ik_solver_ptr->getNumWorkspacePoints()
                       << " workspace points.\n";
        } else {
            LOG_INFO()
                << "Workspace directory exists but no valid data found.\n";
        }
    } catch (const std::exception &e) {
        LOG_INFO() << "Failed to initialize workspace-based IK solver: "
                   << e.what() << "\n";
        LOG_INFO() << "Using fallback IK solver without workspace support.\n";
        workspace_available = false;
    }

    LOG_INFO() << "Waiting for DDS PUB\n";

    // Frequency measurement helpers
    auto last_time = std::chrono::steady_clock::now();
    int sample_count = 0;

    // Joint angle array for motor commands
    double joint_angles[MotorController::getMOTORS()];

    while (true) {
        // 5) Take all available DDS samples
        auto samples = reader.take();
        for (auto const &sample : samples) {
            // Skip invalid samples (e.g. disposals)
            if (!sample.info().valid()) {
                continue;
            }

            ++sample_count;

            // Ensure motors are safe before applying new pose
            controller.ReleaseArmTorque();

            auto const &cmd = sample.data();
            const auto &header = cmd.header();
            const auto &pose = cmd.pose();

            // Log the received pose
            LOG_INFO() << "Sample " << sample_count << " received: ["
                       << pose.position().x() << ", " << pose.position().y()
                       << ", " << pose.position().z() << ", "
                       << pose.orientation().x() << ", "
                       << pose.orientation().y() << ", "
                       << pose.orientation().z() << ", "
                       << pose.orientation().w() << "]\n";

            // Convert pose to Eigen::Vector<double, 7> format [x, y, z, qx, qy,
            // qz, qw]
            Eigen::Vector<double, 7> target_pose_xyzquat;
            target_pose_xyzquat << pose.position().x(), pose.position().y(),
                pose.position().z(), pose.orientation().x(),
                pose.orientation().y(), pose.orientation().z(),
                pose.orientation().w();

            // Solve IK using the IK solver
            JointConfig solution;
            try {
                if (workspace_available && ik_solver_ptr != nullptr) {
                    // Use workspace-based IK solver
                    solution = ik_solver_ptr->solve(target_pose_xyzquat);
                    LOG_INFO() << "IK solution found successfully using "
                                  "workspace solver!\n";
                } else {
                    throw std::runtime_error(
                        "Workspace-based IK solver is not available. No "
                        "fallback permitted.");
                }

                // Convert solution to joint angles array
                for (int i = 0;
                     i < MotorController::getMOTORS() && i < solution.size();
                     i++) {
                    joint_angles[i] = solution(i);
                }

            } catch (const std::exception &e) {
                LOG_INFO() << "IK solving failed: " << e.what()
                           << ". Using neutral configuration.\n";

                // Fallback to neutral configuration if IK fails
                for (int i = 0; i < MotorController::getMOTORS(); i++) {
                    joint_angles[i] = 0.0; // Neutral pose as fallback
                }
            }

            // Send pose to motor controller
            controller.SetArmPose(joint_angles, ACCEL_LIMIT);

            // Log applied joint angles
            LOG_INFO() << "Applied targets ";
            for (double q : joint_angles)
                std::cout << ' ' << q;
            std::cout << '\n';
        }

        // 7) Print message receive frequency every 100 samples
        if (sample_count >= 100) {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time).count();
            double freq = sample_count / dt;
            std::cout << "[IK SUB] Receive frequency ≈ " << freq << " Hz\n";
            last_time = now;
            sample_count = 0;
        }

        // Release torque after iteration
        std::chrono::milliseconds(
            10); // (has no effect here; line can be removed)
        // controller.ReleaseArmTorque();

        // 8) Sleep to avoid busy-looping
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Cleanup
    if (ik_solver_ptr != nullptr) {
        delete ik_solver_ptr;
    }

    return 0;
}
