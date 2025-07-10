// motor_controller.h
#pragma once

#include <vector>
#include <mutex>
#include <array>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <limits>

#include "../build/_deps/moteus-src/lib/cpp/mjbots/moteus/moteus.h"

using namespace mjbots;

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

    static moteus::Controller::Options createOptions(u_int8_t id)
    {

        moteus::Controller::Options options;
        options.id = id;
        options.position_format.accel_limit = moteus::kFloat;
        options.position_format.maximum_torque = moteus::kFloat;
        options.query_format.trajectory_complete = moteus::kInt8;
        options.query_format.q_current = moteus::kFloat;
        options.query_format.d_current = moteus::kFloat;
        options.query_format.abs_position = moteus::kFloat;
        options.query_format.power = moteus::kFloat;
        // This will only be valid if an NTC thermistor is connected to the TEMP pads, motor.thermistor_ohm is set correctly, and servo.enable_motor_temperature is enabled.
        //  options.query_format.motor_temperature = moteus::kFloat;
        options.query_format.aux1_gpio = moteus::kInt8;
        options.query_format.aux2_gpio = moteus::kInt8;
        return options;
    }

    /// Underlying Moteus controller objects for each motor.
    std::vector<moteus::Controller> controllers_;

    /* Holds status for each motor using the predefined Result struct from the moteus lib. It holds the following data -
        Mode mode = Mode::kStopped;
        double position = NaN;
        double velocity = NaN;
        double torque = NaN;
        double q_current = NaN;
        double d_current = NaN;
        double abs_position = NaN;
        double power = NaN;
        double motor_temperature = NaN;
        bool trajectory_complete = false;
        HomeState home_state = HomeState::kRelative;
        double voltage = NaN;
        double temperature = NaN;
        int8_t fault = 0;

        int8_t aux1_gpio = 0;
        int8_t aux2_gpio = 0;
    */
    std::array<moteus::Query::Result, MOTORS> motor_statuses_;

    // In motor_controller.h, inside class MotorController:
    std::thread worker_thread_;
    mutable std::mutex command_mutex_;
    std::condition_variable command_cv_;
    bool stop_thread_ = false;
    bool has_new_command_ = false;
    std::array<double, MOTORS> next_positions_{};
    double next_accel_ = 1.0;
    std::atomic<bool> trajectory_complete_{false};

    // Main persistent thread loop
    void WorkerLoop();

    // The workhorse that does the hammer-and-poll for one command
    void DoSetMotorPositions_(const std::array<double, MOTORS> &positions,
                              double accel_limit);

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
    void SetMotorPosition(double pos, double accel_limit = 1.0, unsigned id = 0);

    /**
     * @brief Moves all motors to specified joint positions (“arm pose”).
     * @param pos Array of target positions (radians) of length MOTORS.
     * @param accel_limit Acceleration limit applied to all motors.
     */

    /// Enqueue a new simultaneous move.
    /// Returns immediately; the worker thread will hammer the new positions.
    void SetMotorPositions(const std::array<double, MOTORS> &positions,
                           double accel_limit = 1.0);

    /// Query whether the last move has completed (all motors report trajectory_complete)
    bool getTrajectoryComplete() const noexcept { return trajectory_complete_.load(); }

    /**
     * @brief Returns the number of motors managed by this controller.
     * @return The constant MOTORS value.
     */
    static constexpr unsigned getMOTORS() { return MOTORS; }

    /**
     * @brief Immediately stops (releases torque on) a single motor.
     * @param id Motor index.
     */
    inline void SetMotorStop(unsigned id)
    {
        motor_statuses_[id] = controllers_[id].SetStop()->values;
    }

    /**
     * @brief Immediately stops (releases torque on) all motors.
     */
    inline void SetMotorsStop()
    {
        for (int i = 0; i < MOTORS; i++)
            motor_statuses_[i] = controllers_[i].SetStop()->values;
    }
    inline const std::array<mjbots::moteus::Query::Result, MOTORS> &
    getMotorStatuses() const noexcept { return motor_statuses_; }

    inline void RefreshMotorStatuses()
    {
        for (size_t i = 0; i < MOTORS; ++i)
        {
            if (auto maybe = controllers_[i].SetQuery())
            {
                motor_statuses_[i] = maybe->values;
            }
        }
    }
};
