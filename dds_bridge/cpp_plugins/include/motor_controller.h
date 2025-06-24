#pragma once

#include <vector>
#include "../build/_deps/moteus-src/lib/cpp/mjbots/moteus/moteus.h"
#include <mutex>
#include <array>

class MotorController
{
private:
    static constexpr unsigned MOTORS = 2;
    // Cleanly stop/join any active hold threads
    void stopHoldThreads();

    const std::array<double, MOTORS> MAX_TORQUE;
    static std::array<double, MOTORS> init_MAX_TORQUE()
    {
        std::array<double, MOTORS> a;
        a.fill(std::numeric_limits<double>::quiet_NaN());
        return a;
    }

    std::vector<mjbots::moteus::Controller> controllers_;

    // For our 2-thread time-slice hold
    std::vector<std::thread> hold_threads_;
    std::atomic<bool> hold_running_{false};
    std::mutex hold_mutex_;
    std::array<std::mutex, MOTORS> controller_mutexes_;

public:
    MotorController();
    // Construct 6 controllers with IDs 0…5
    MotorController(const std::array<double, MOTORS> &max_torque);

    ~MotorController();

    // Move motor `id` (0–5) to position `pos`
    void SetPosition(double pos, double accel_limit = 1.0, unsigned id = 0);

    void SetArmPose(const double pos[MOTORS], const double accel_limit = 1.0);

    static constexpr unsigned getMOTORS() { return MOTORS; }

    void ReleaseMotorTorque(unsigned id);

    void ReleaseArmTorque();
};
