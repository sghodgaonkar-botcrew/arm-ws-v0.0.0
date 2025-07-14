/**
 * @file motor_controller.h
 * @brief Multi-motor position controller using Moteus motor controllers
 * @author Shantanu Ghodgaonkar (sghodgaonkar@botcrew.com)
 * @date 2025-07-14
 *
 * @details
 * This class provides a thread-safe interface for controlling multiple Moteus
 * motor controllers simultaneously. It implements a producer-consumer pattern
 * where:
 * - The main thread enqueues position commands via SetMotorPositions()
 * - A dedicated worker thread continuously sends commands to motors at ~100Hz
 * - The worker thread handles fault detection, frame drops, and trajectory
 * completion
 *
 * Control Flow:
 * 1. Constructor initializes motor controllers and starts worker thread
 * 2. SetMotorPositions() enqueues new position targets
 * 3. Worker thread continuously sends position commands to all motors
 * 4. Worker thread monitors motor status for faults and trajectory completion
 * 5. Destructor gracefully shuts down worker thread
 *
 * Threading Model:
 * - Main thread: Enqueues commands, queries status
 * - Worker thread: Continuous motor control loop
 * - Synchronization: Mutex + condition variable for command queue
 * - Atomic variables for thread-safe status queries
 */

// motor_controller.h
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include "../build/_deps/moteus-src/lib/cpp/mjbots/moteus/moteus.h"

using namespace mjbots;

/**
 * @class MotorController
 * @brief Manages multiple Moteus motor controllers, providing position control,
 *        torque release, and a threaded hold mechanism to maintain an arm pose.
 *
 * @details
 * This class implements a robust multi-motor control system with the following
 * features:
 *
 * - **Thread-Safe Operation**: Uses mutex and condition variables for safe
 * command enqueueing
 * - **Continuous Control Loop**: Worker thread runs at ~100Hz to maintain
 * position commands
 * - **Fault Detection**: Monitors motor faults and position timeouts
 * - **Frame Drop Recovery**: Implements retry logic for dropped communication
 * frames
 * - **Trajectory Tracking**: Tracks completion status of position trajectories
 * - **Graceful Shutdown**: Properly terminates worker thread on destruction
 *
 * The control architecture follows a producer-consumer pattern:
 * - Producer (main thread): Calls SetMotorPositions() to enqueue new targets
 * - Consumer (worker thread): Continuously sends commands and monitors status
 *
 * @note All position values are in radians, but are converted to revolutions
 * for Moteus controllers
 * @note The worker thread runs continuously until the object is destroyed
 * @note Frame drops are handled with retry logic, with emergency stop after 5
 * consecutive failures
 */
class MotorController {
private:
  /// Number of motors managed by this controller.
  static constexpr unsigned MOTORS = 2;

  /// Maximum torque limits for each motor. NaN indicates "no limit."
  const std::array<double, MOTORS> MAX_TORQUE;

  /**
   * @brief Helper to initialize MAX_TORQUE with NaN values.
   * @param mt Torque value to fill array with (defaults to NaN for no limit)
   * @return Array filled with specified torque value for each motor slot.
   *
   * @details
   * Creates a std::array<double, MOTORS> filled with the specified torque
   * value. When mt is NaN (default), it indicates no torque limit for safety.
   */
  static std::array<double, MOTORS>
  init_MAX_TORQUE(double mt = std::numeric_limits<double>::quiet_NaN()) {
    std::array<double, MOTORS> a;
    a.fill(mt);
    return a;
  }

  /**
   * @brief Creates Moteus controller configuration options for a specific motor
   * ID.
   * @param id The motor ID (0-based index)
   * @return Configured Moteus controller options
   *
   * @details
   * Sets up the communication format for both position commands and status
   * queries. Configures which data fields are included in position commands and
   * status responses. This ensures efficient communication by only transferring
   * necessary data.
   */
  static moteus::Controller::Options createOptions(u_int8_t id) {
    moteus::Controller::Options options;
    options.id = id;

    // Position command format - specify which fields to include
    options.position_format.accel_limit =
        moteus::kFloat; // Include acceleration limit
    options.position_format.maximum_torque =
        moteus::kFloat; // Include torque limit
    options.position_format.watchdog_timeout =
        moteus::kFloat; // Include watchdog timeout

    // Status query format - specify which fields to request
    options.query_format.trajectory_complete =
        moteus::kInt8; // Trajectory completion status
    options.query_format.q_current = moteus::kFloat;    // Q-axis current
    options.query_format.d_current = moteus::kFloat;    // D-axis current
    options.query_format.abs_position = moteus::kFloat; // Absolute position
    options.query_format.power = moteus::kFloat;        // Power consumption

    // Temperature monitoring (commented out - requires hardware setup)
    // This will only be valid if an NTC thermistor is connected to the TEMP
    // pads, motor.thermistor_ohm is set correctly, and
    // servo.enable_motor_temperature is enabled.
    // options.query_format.motor_temperature = moteus::kFloat;

    // GPIO status
    options.query_format.aux1_gpio = moteus::kInt8; // Auxiliary GPIO 1
    options.query_format.aux2_gpio = moteus::kInt8; // Auxiliary GPIO 2

    return options;
  }

  /// Underlying Moteus controller objects for each motor.
  std::vector<moteus::Controller> controllers_;

  /**
   * @brief Motor status storage for each motor.
   *
   * @details
   * Holds status for each motor using the predefined Result struct from the
   * moteus lib. It contains the following data:
   * - Mode mode = Mode::kStopped;           // Current motor mode
   * - double position = NaN;                // Current position (revolutions)
   * - double velocity = NaN;                // Current velocity
   * - double torque = NaN;                  // Current torque
   * - double q_current = NaN;               // Q-axis current
   * - double d_current = NaN;               // D-axis current
   * - double abs_position = NaN;            // Absolute position
   * - double power = NaN;                   // Power consumption
   * - double motor_temperature = NaN;       // Motor temperature
   * - bool trajectory_complete = false;     // Trajectory completion flag
   * - HomeState home_state = HomeState::kRelative; // Homing state
   * - double voltage = NaN;                 // Supply voltage
   * - double temperature = NaN;             // Controller temperature
   * - int8_t fault = 0;                     // Fault code
   * - int8_t aux1_gpio = 0;                 // Auxiliary GPIO 1
   * - int8_t aux2_gpio = 0;                 // Auxiliary GPIO 2
   */
  std::array<moteus::Query::Result, MOTORS> motor_statuses_;

  // Threading and synchronization members
  /// Worker thread that continuously sends motor commands
  std::thread worker_thread_;

  /// Mutex for protecting command queue and shared state
  mutable std::mutex command_mutex_;

  /// Condition variable for signaling new commands to worker thread
  std::condition_variable command_cv_;

  /// Flag to signal worker thread to stop (set in destructor)
  bool stop_thread_ = false;

  /// Flag indicating new command is available for worker thread
  bool has_new_command_ = false;

  /// Next position targets for each motor (radians)
  std::array<double, MOTORS> next_positions_{};

  /// Next acceleration limit for position commands
  double next_accel_ = 1.0;

  /// Atomic flag indicating if current trajectory is complete
  std::atomic<bool> trajectory_complete_{false};

  /**
   * @brief Main worker thread function that continuously controls motors.
   *
   * @details
   * This function implements the core control loop that:
   * 1. Waits for the first command via condition variable
   * 2. Continuously sends position commands to all motors at ~100Hz
   * 3. Monitors motor status for faults and trajectory completion
   * 4. Handles frame drops with retry logic
   * 5. Checks for new commands and updates targets accordingly
   * 6. Runs until stop_thread_ is set to true
   *
   * The loop operates in two phases:
   * - Initialization: Waits for first command, then drops mutex lock
   * - Main loop: Continuously sends commands, checks status, handles new
   * commands
   *
   * Safety features:
   * - Emergency stop on motor faults or position timeouts
   * - Emergency stop after 5 consecutive frame drops
   * - Graceful shutdown when stop_thread_ is set
   */
  void WorkerLoop();

public:
  /**
   * @brief Default constructor.
   *
   * @details
   * Initializes the motor controller with the following sequence:
   * 1. Sets MAX_TORQUE to NaN (no torque limits)
   * 2. Reserves space for MOTORS controllers
   * 3. Creates Moteus controller objects with default options
   * 4. Stops all motors to ensure clean initial state
   * 5. Starts the worker thread for continuous control
   *
   * @note The worker thread will wait for the first command before starting
   * control
   */
  MotorController();

  /**
   * @brief Constructor with explicit torque limits.
   * @param max_torque Array of maximum torque values per motor.
   *
   * @details
   * Same initialization as default constructor, but with explicit torque
   * limits. Each motor will be limited to the specified torque value for
   * safety.
   *
   * @note Torque values should be positive and reasonable for the motor
   * hardware
   */
  MotorController(const double &max_torque);

  /**
   * @brief Destructor.
   *
   * @details
   * Performs graceful shutdown of the motor controller:
   * 1. Sets stop_thread_ flag to signal worker thread to stop
   * 2. Notifies worker thread via condition variable
   * 3. Waits for worker thread to complete (join)
   * 4. Releases all resources
   *
   * @note This ensures the worker thread is properly terminated before
   * destruction
   */
  ~MotorController();

  /**
   * @brief Enqueues a new simultaneous move for all motors.
   * @param positions Array of target positions in radians for each motor
   * @param accel_limit Acceleration limit applied to all motors (default: 1.0)
   *
   * @details
   * This function implements the producer side of the producer-consumer
   * pattern:
   * 1. Acquires mutex lock to protect shared command state
   * 2. Updates next_positions_ and next_accel_ with new targets
   * 3. Sets has_new_command_ flag to true
   * 4. Resets trajectory_complete_ flag to false
   * 5. Notifies worker thread via condition variable
   * 6. Returns immediately (non-blocking)
   *
   * The worker thread will pick up these new targets in its next iteration.
   *
   * @note This function is thread-safe and can be called from any thread
   * @note NaN values in positions array will disable position control for that
   * motor
   * @note The function returns immediately; use getTrajectoryComplete() to
   * check completion
   */
  void SetMotorPositions(const std::array<double, MOTORS> &positions,
                         double accel_limit = 1.0);

  /**
   * @brief Queries whether the last move has completed.
   * @return true if all motors report trajectory_complete, false otherwise
   *
   * @details
   * Returns the current state of the trajectory completion flag.
   * This flag is set to true when all motors report that their position
   * trajectories have completed successfully.
   *
   * @note This is an atomic operation and is thread-safe
   * @note The flag is automatically reset to false when new commands are
   * enqueued
   */
  bool getTrajectoryComplete() const noexcept {
    return trajectory_complete_.load();
  }

  /**
   * @brief Returns the number of motors managed by this controller.
   * @return The constant MOTORS value (2).
   *
   * @details
   * Provides access to the number of motors for external code that needs
   * to know the system configuration.
   */
  static constexpr unsigned getMOTORS() { return MOTORS; }

  /**
   * @brief Immediately stops (releases torque on) a single motor.
   * @param id Motor index (0-based)
   *
   * @details
   * Sends an immediate stop command to the specified motor, which:
   * 1. Releases all torque on the motor
   * 2. Updates the motor status with the stop response
   *
   * @note This is a blocking call that waits for the stop command to complete
   * @note The motor will coast to a stop (no active braking)
   */
  inline void SetMotorStop(unsigned id) {
    motor_statuses_[id] = controllers_[id].SetStop()->values;
  }

  /**
   * @brief Immediately stops (releases torque on) all motors.
   *
   * @details
   * Sends immediate stop commands to all motors simultaneously:
   * 1. Acquires mutex lock to ensure thread safety
   * 2. Sends stop command to each motor
   * 3. Updates motor statuses with stop responses
   *
   * @note This is a blocking call that waits for all stop commands to complete
   * @note All motors will coast to a stop (no active braking)
   * @note This function is thread-safe
   */
  inline void SetMotorsStop() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    for (int i = 0; i < MOTORS; i++)
      motor_statuses_[i] = controllers_[i].SetStop()->values;
  }

  /**
   * @brief Returns a reference to the current motor statuses.
   * @return Const reference to array of motor status structures
   *
   * @details
   * Provides read-only access to the current status of all motors.
   * The returned array contains the most recent status information
   * from each motor, including position, velocity, torque, etc.
   *
   * @note This is thread-safe as it returns a const reference
   * @note Status information is updated by the worker thread during normal
   * operation
   */
  inline const std::array<mjbots::moteus::Query::Result, MOTORS> &
  getMotorStatuses() const noexcept {
    return motor_statuses_;
  }

  /**
   * @brief Manually refreshes motor status information.
   *
   * @details
   * Forces a status query to all motors to update the motor_statuses_ array.
   * This can be useful for getting the most current status when the worker
   * thread might not be actively controlling the motors.
   *
   * @note This function queries each motor individually and may take some time
   * @note Status queries are sent even if the motor is not in position mode
   */
  inline void RefreshMotorStatuses() {
    // Note: No mutex lock here as this is typically called when worker thread
    // is inactive
    for (size_t i = 0; i < MOTORS; ++i) {
      if (auto maybe = controllers_[i].SetQuery()) {
        motor_statuses_[i] = maybe->values;
      }
    }
  }
};
