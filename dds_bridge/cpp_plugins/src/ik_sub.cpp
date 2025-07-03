#include <iostream>
#include "arm_msgs.hpp"
#include "motor_controller.h"
#include <thread>
#include <chrono>
#include <dds/dds.hpp>
#include <iomanip>
#include "ik_model.h"

/**
 * @brief Returns the current time in seconds with nanosecond precision.
 * 
 * @return double Time since epoch in seconds.
 */
double now_seconds()
{
    using namespace std::chrono;
    auto t = high_resolution_clock::now().time_since_epoch();
    return duration<double>(t).count();
}

/// @brief Macro to log INFO messages with a timestamp prefix and a fixed precision.
#define LOG_INFO() \
    std::cout << "[INFO] [" << std::fixed << std::setprecision(9) << now_seconds() << "] [IK SUB] "

/// @brief Placeholder acceleration limit value for joint movement
#define ACCEL_LIMIT NaN

/**
 * @brief Main entry point for the IK subscriber executable.
 * 
 * Subscribes to a DDS topic `ArmCommandTopic`, receives pose commands,
 * computes joint angles (placeholder logic here), and sends commands to a motor controller.
 */
int main(int argc, char **argv)
{
    // Initialize DDS participant and topic
    dds::domain::DomainParticipant participant(0);
    dds::topic::Topic<arm_msgs::ArmCommand> topic(participant, "ArmCommandTopic");

    // Create DDS subscriber with QoS policies
    auto subscriber = dds::sub::Subscriber(participant);
    auto dr_qos = subscriber.default_datareader_qos()
                  << dds::core::policy::Reliability::Reliable()
                  // << dds::core::policy::History::KeepLast(10)
                  << dds::core::policy::Durability::Volatile();
    dds::sub::DataReader<arm_msgs::ArmCommand> reader(subscriber, topic, dr_qos);

    // Instantiate the motor controller
    MotorController controller;

    LOG_INFO() << "Waiting for DDS PUB\n";

    // Frequency measurement helpers
    auto last_time = std::chrono::steady_clock::now();
    int sample_count = 0;

    // Joint angle array for motor commands
    double joint_angles[MotorController::getMOTORS()];

    while (true)
    {
        // 5) Take all available DDS samples
        auto samples = reader.take();
        for (auto const &sample : samples)
        {
            // Skip invalid samples (e.g. disposals)
            if (!sample.info().valid())
            {
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
                       << pose.position().x() << ", "
                       << pose.position().y() << ", "
                       << pose.position().z() << ", "
                       << pose.orientation().x() << ", "
                       << pose.orientation().y() << ", "
                       << pose.orientation().z() << ", "
                       << pose.orientation().w() << "]\n";

            // IK Solver stub — populate joint_angles based on received pose
            if (pose.position().x() == 0.0)
            {
                for (int i = 0; i < MotorController::getMOTORS(); i++)
                    joint_angles[i] = 0.0;  // Example static pose
            }
            else
            {
                for (int i = 0; i < MotorController::getMOTORS(); i += 2)
                    joint_angles[i] = 5.0;   // Example alternate pose

                for (int i = 1; i < MotorController::getMOTORS(); i += 2)
                    joint_angles[i] = -5.0;
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
        if (sample_count >= 100)
        {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time).count();
            double freq = sample_count / dt;
            std::cout << "[IK SUB] Receive frequency ≈ " << freq << " Hz\n";
            last_time = now;
            sample_count = 0;
        }

        // Release torque after iteration
        std::chrono::milliseconds(10);  // (has no effect here; line can be removed)
        // controller.ReleaseArmTorque();

        // 8) Sleep to avoid busy-looping
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
