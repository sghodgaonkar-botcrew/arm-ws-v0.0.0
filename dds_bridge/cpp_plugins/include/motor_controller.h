// motor_controller.h
#pragma once

#include <vector>
#include <mutex>
#include <array>
#include "../build/_deps/moteus-src/lib/cpp/mjbots/moteus/moteus.h"

/**
 * @class MotorController
 * @brief Manages multiple Moteus motor controllers, providing position control,
 *        torque release, and a threaded hold mechanism to maintain an arm pose.
 */
class MotorController
{
private:
    /// Number of motors managed by this controller.
    static constexpr unsigned MOTORS = 2;

    /**
     * @brief Cleanly stops and joins any active hold threads.
     *
     * Called internally to shut down the time‐slice hold threads before
     * modifying poses or destroying the object.
     */
    void stopHoldThreads();

    /// Maximum torque limits for each motor. NaN indicates “no limit.”
    const std::array<double, MOTORS> MAX_TORQUE;

    /**
     * @brief Helper to initialize MAX_TORQUE with NaN values.
     * @return Array filled with NaN for each motor slot.
     */
    static std::array<double, MOTORS> init_MAX_TORQUE()
    {
        std::array<double, MOTORS> a;
        a.fill(std::numeric_limits<double>::quiet_NaN());
        return a;
    }

    /// Underlying Moteus controller objects for each motor.
    std::vector<mjbots::moteus::Controller> controllers_;

    /// Threads used to implement a time‐slice “hold pose” across motors.
    std::vector<std::thread> hold_threads_;

    /// Flag indicating whether hold threads should continue running.
    std::atomic<bool> hold_running_{false};

    /// Mutex protecting access to hold_threads_ and hold_running_.
    std::mutex hold_mutex_;

    /// Mutex array to synchronize each controller during multi‐threaded holds.
    std::array<std::mutex, MOTORS> controller_mutexes_;

public:
    /**
     * @brief Default constructor.
     *
     * Initializes controllers_ with default options, sets MAX_TORQUE to NaN,
     * and prepares internal data structures for holding poses.
     */
    MotorController();

    /**
     * @brief Constructor with explicit torque limits.
     * @param max_torque Array of maximum torque values per motor.
     */
    MotorController(const std::array<double, MOTORS> &max_torque);

    /**
     * @brief Destructor.
     *
     * Stops any running hold threads and releases resources before object
     * destruction.
     */
    ~MotorController();

    /**
     * @brief Moves a single motor to a target position.
     * @param pos Target position in radians.
     * @param accel_limit Acceleration limit in revolutions/sec².
     * @param id Motor index [0 … MOTORS-1].
     */
    void SetPosition(double pos, double accel_limit = 1.0, unsigned id = 0);

    /**
     * @brief Moves all motors to specified joint positions (“arm pose”).
     * @param pos Array of target positions (radians) of length MOTORS.
     * @param accel_limit Acceleration limit applied to all motors.
     */
    void SetArmPose(const double pos[MOTORS], const double accel_limit = 1.0);

    /**
     * @brief Returns the number of motors managed by this controller.
     * @return The constant MOTORS value.
     */
    static constexpr unsigned getMOTORS() { return MOTORS; }

    /**
     * @brief Immediately stops (releases torque on) a single motor.
     * @param id Motor index.
     */
    void ReleaseMotorTorque(unsigned id);

    /**
     * @brief Immediately stops (releases torque on) all motors.
     */
    void ReleaseArmTorque();
};
