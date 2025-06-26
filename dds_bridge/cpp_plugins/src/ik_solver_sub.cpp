#include <iostream>
#include "arm_msgs.hpp"
#include "motor_controller.h"
#include <thread>
#include <chrono>
#include <dds/dds.hpp>
#include <iomanip>

double now_seconds()
{
    using namespace std::chrono;
    // you can choose system_clock or steady_clock
    auto t = high_resolution_clock::now().time_since_epoch();
    return duration<double>(t).count();
}

#define LOG_INFO() \
    std::cout << "[INFO] [" << std::fixed << std::setprecision(9) << now_seconds() << "] [IK SUB] "

#define ACCEL_LIMIT NaN

int main(int argc, char **argv)
{
    dds::domain::DomainParticipant participant(0);
    dds::topic::Topic<arm_msgs::ArmCommand> topic(participant, "ArmCommandTopic");

    auto subscriber = dds::sub::Subscriber(participant);
    auto dr_qos = subscriber.default_datareader_qos()
                  << dds::core::policy::Reliability::Reliable()
                  // << dds::core::policy::History::KeepLast(10)
                  << dds::core::policy::Durability::Volatile();
    dds::sub::DataReader<arm_msgs::ArmCommand> reader(subscriber, topic, dr_qos);
    MotorController controller;
    LOG_INFO() << "Waiting for DDS PUB\n";
    // Variables for measuring receive frequency
    auto last_time = std::chrono::steady_clock::now();
    int sample_count = 0;
    double joint_angles[MotorController::getMOTORS()];
    while (true)
    {
        // 5) Take all available samples
        auto samples = reader.take();
        for (auto const &sample : samples)
        {

            if (!sample.info().valid())
            {
                continue; // skip disposals, etc.
            }

            ++sample_count;
            controller.ReleaseArmTorque();
            auto const &cmd = sample.data();
            // Convert IDL std::vector to std::vector<double>
            const auto &header = cmd.header();
            const auto &pose = cmd.pose();

            LOG_INFO() << "Sample " << sample_count << " received: ["
                       << pose.position().x() << ", "
                       << pose.position().y() << ", "
                       << pose.position().z() << ", "
                       << pose.orientation().x() << ", "
                       << pose.orientation().y() << ", "
                       << pose.orientation().z() << ", "
                       << pose.orientation().w() << "]\n";
            if (pose.position().x() == 0.0)
            {
                // Apply End Effector Pose to IK Solver and get joint angles
                for (int i = 0; i < MotorController::getMOTORS(); i++)
                    joint_angles[i] = // IK solution
                        0.0;
            }
            else
            {
                for (int i = 0; i < MotorController::getMOTORS(); i += 2)
                    joint_angles[i] = // IK solution
                        5.0;
                for (int i = 1; i < MotorController::getMOTORS(); i += 2)
                    joint_angles[i] = // IK solution
                        -5.0;
            }

            controller.SetArmPose(joint_angles, ACCEL_LIMIT);

            // Print the applied targets
            LOG_INFO() << "Applied targets ";
            for (double q : joint_angles)
                std::cout << ' ' << q;
            std::cout << '\n';
        }

        // 7) Frequency measurement (every 100 samples)
        if (sample_count >= 100)
        {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last_time).count();
            double freq = sample_count / dt;
            std::cout << "[IK SUB] Receive frequency ≈ " << freq << " Hz\n";
            last_time = now;
            sample_count = 0;
        }
        std::chrono::milliseconds(10);
        controller.ReleaseArmTorque();
        // 8) Sleep to avoid busy-looping
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return 0;
}