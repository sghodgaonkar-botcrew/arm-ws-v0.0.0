/**
 * @file motor_controller.cpp
 * @brief Implementation of the MotorController class for multi-motor position
 * control
 * @author Shantanu Ghodgaonkar (sghodgaonkar@botcrew.com)
 * @date 2025-07-14
 *
 * @details
 * This file implements the MotorController class which provides thread-safe
 * control of multiple Moteus motor controllers. The implementation uses a
 * producer-consumer pattern with a dedicated worker thread for continuous
 * motor control at ~100Hz.
 *
 * Key Implementation Details:
 * - Worker thread runs continuously until object destruction
 * - Position commands are sent at ~100Hz to maintain motor position
 * - Fault detection and frame drop recovery are handled automatically
 * - Thread synchronization uses mutex + condition variable pattern
 * - All position values are converted from radians to revolutions for Moteus
 */

#include "../include/motor_controller.h"
#include <chrono>
#include <cmath> // for std::round
#include <iostream>

namespace {
/// Mathematical constant π.
constexpr double PI = 3.141592653589793;

/**
 * @brief Converts radians to revolutions.
 * @param r Angle in radians.
 * @return Equivalent revolutions.
 *
 * @details
 * Moteus controllers expect position commands in revolutions, but this
 * interface uses radians for consistency with typical robotics applications.
 * This function performs the conversion: revolutions = radians / (2π)
 */
static inline double rad2rev(double r) { return r / (2 * PI); }

/**
 * @brief Checks if two values are within a given tolerance.
 * @param a First value.
 * @param b Second value.
 * @param tol Allowed tolerance (default: 0.005).
 * @return True if |a - b| ≤ tol.
 *
 * @details
 * Utility function for comparing floating-point values with a tolerance.
 * Uses absolute difference comparison to handle floating-point precision
 * issues. Default tolerance of 0.005 radians (~0.29 degrees) is suitable for
 * most motor position comparisons.
 */
static inline bool close_enough(double a, double b, double tol = 0.005) {
  return std::fabs(a - b) <= tol;
}
} // namespace

using namespace mjbots; ///< Bring moteus Controller types into scope.

////////////////////////////////////////////////////////////////////////////////
// Constructors / Destructor
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default constructs MotorController with NaN torque limits.
 *
 * @details
 * Initializes each Moteus controller with default options, then stops
 * and queries each to ensure clean state. The initialization sequence is:
 *
 * 1. Initialize MAX_TORQUE with NaN values (no torque limits)
 * 2. Reserve space for MOTORS controllers in the vector
 * 3. Create Moteus controller objects with default options for each motor
 * 4. Send stop command to each motor to ensure clean initial state
 * 5. Start the worker thread for continuous motor control
 *
 * The worker thread will wait for the first command before beginning control.
 */
MotorController::MotorController() : MAX_TORQUE(init_MAX_TORQUE()) {
  // Reserve space for all motor controllers to avoid reallocation
  controllers_.reserve(MOTORS);

  // Initialize each motor controller
  for (int i = 0; i < MOTORS; ++i) {
    // Create controller with default options for this motor ID
    controllers_.emplace_back(createOptions(i));

    // Send stop command and capture initial status
    // This ensures motors start in a known, safe state
    motor_statuses_[i] = controllers_[i].SetStop()->values;
  }

  // Start the worker thread for continuous motor control
  // The thread will wait for the first command before beginning control
  worker_thread_ = std::thread(&MotorController::WorkerLoop, this);
}

/**
 * @brief Constructs MotorController with explicit torque limits.
 * @param max_torque Array specifying maximum torque per motor.
 *
 * @details
 * Same initialization as default constructor, but with explicit torque limits
 * for safety. Each motor will be limited to the specified torque value to
 * prevent damage to the hardware or unexpected behavior.
 */
MotorController::MotorController(const double &max_torque)
    : MAX_TORQUE{init_MAX_TORQUE(max_torque)} {
  // Reserve space for all motor controllers to avoid reallocation
  controllers_.reserve(MOTORS);

  // Initialize each motor controller
  for (int i = 0; i < MOTORS; ++i) {
    // Create controller with default options for this motor ID
    controllers_.emplace_back(createOptions(i));

    // Send stop command and capture initial status
    // This ensures motors start in a known, safe state
    motor_statuses_[i] = controllers_[i].SetStop()->values;
  }

  // Start the worker thread for continuous motor control
  // The thread will wait for the first command before beginning control
  worker_thread_ = std::thread(&MotorController::WorkerLoop, this);
}

/**
 * @brief Destructor stops any running hold threads and cleans up.
 *
 * @details
 * Performs graceful shutdown of the motor controller system:
 * 1. Acquires mutex lock to safely modify shared state
 * 2. Sets stop_thread_ flag to signal worker thread to terminate
 * 3. Notifies worker thread via condition variable to wake it up
 * 4. Releases mutex lock
 * 5. Waits for worker thread to complete (join)
 *
 * This ensures the worker thread is properly terminated before the object
 * is destroyed, preventing any race conditions or resource leaks.
 */
MotorController::~MotorController() {
  { // tell the worker to stop
    std::lock_guard<std::mutex> lock(command_mutex_);
    stop_thread_ = true;
    command_cv_.notify_one();
  }
  worker_thread_.join();
  // Note: SetMotorsStop() is not called here as the worker thread
  // should have already stopped all motors during its shutdown
}

/**
 * @brief Main worker thread function that continuously controls motors.
 *
 * @details
 * This function implements the core control loop that runs continuously
 * until the object is destroyed. The loop operates in two distinct phases:
 *
 * Phase 1 - Initialization:
 * - Waits for the first command via condition variable
 * - Grabs initial command parameters and drops mutex lock permanently
 * - Sends stop command to all motors to ensure clean state
 *
 * Phase 2 - Main Control Loop:
 * - Continuously sends position commands to all motors at ~100Hz
 * - Monitors motor status for faults and trajectory completion
 * - Handles frame drops with retry logic and emergency stops
 * - Checks for new commands and updates targets accordingly
 * - Runs until stop_thread_ is set to true
 *
 * Safety Features:
 * - Emergency stop on motor faults or position timeouts
 * - Emergency stop after 5 consecutive frame drops per motor
 * - Graceful shutdown when stop_thread_ is set
 * - Continuous monitoring of motor status
 *
 * Communication Protocol:
 * - Position commands are sent at ~100Hz (10ms intervals)
 * - Each command includes position, acceleration limit, and torque limit
 * - Status responses are captured and stored in motor_statuses_
 * - Frame drops are detected and handled with retry logic
 *
 * @note This function never returns except when stop_thread_ is true
 * @note The function drops the mutex lock after initialization for performance
 * @note All motor communication happens in this thread for thread safety
 */
void MotorController::WorkerLoop() {
  // Phase 1: Initialization - Wait for first command
  std::unique_lock<std::mutex> lock(command_mutex_);

  // Initialize frame drop counters for each motor
  std::array<uint8_t, MOTORS> frame_drop_counter;
  frame_drop_counter.fill(0);

  // Wait for the very first command or stop signal
  // This ensures we don't start sending commands until the user is ready
  command_cv_.wait(lock, [&] { return has_new_command_ || stop_thread_; });

  // Check if we should stop immediately (destructor called before first
  // command)
  if (stop_thread_)
    return;

  // Grab the first command parameters, then drop the lock permanently
  // From this point on, we never call command_cv_.wait() again
  auto positions = next_positions_;  // Copy current position targets
  auto accel = next_accel_;          // Copy current acceleration limit
  has_new_command_ = false;          // Reset command flag
  trajectory_complete_.store(false); // Reset completion flag
  lock.unlock();                     // Drop lock for performance

  // Ensure all motors are stopped before beginning control
  this->SetMotorsStop();

  // Phase 2: Main Control Loop - Run continuously until stop signal
  // From now on we NEVER call command_cv_.wait() again—
  // we just spin in this loop until stop_thread_ is true:
  while (true) {
    // Step 1: Send position commands to all motors at ~100Hz
    for (size_t i = 0; i < MOTORS; ++i) {
      // Debug output (commented out in production)
      // printf("Motor %d sending position = %.3f traj = %c\n", i, positions[i],
      // motor_statuses_[i].trajectory_complete ? 't' : 'f');

      // Prepare position command for this motor
      moteus::PositionMode::Command cmd;

      // Handle NaN position values (disable position control)
      if (std::isnan(positions[i])) {
        cmd.position = std::numeric_limits<double>::quiet_NaN();
        cmd.velocity = 0.0;
      } else {
        // Convert radians to revolutions for Moteus controller
        cmd.position = rad2rev(positions[i]);
        cmd.accel_limit = accel;

        // Apply torque limit if specified (not NaN)
        if (!std::isnan(MAX_TORQUE[i])) {
          cmd.maximum_torque = MAX_TORQUE[i];
        }
      }

      // Send position command and check for successful response
      auto maybe = controllers_[i].SetPosition(cmd);
      if (maybe) {
        // Command sent successfully - reset frame drop counter and update
        // status
        frame_drop_counter[i] = 0;
        motor_statuses_[i] = maybe->values;

        // Check for motor faults or position timeouts
        if (motor_statuses_[i].fault ||
            (motor_statuses_[i].mode == moteus::Mode::kPositionTimeout)) {
          // Emergency stop: Stop all motors and exit
          for (auto &c : controllers_) {
            c.SetStop();
            moteus::PositionMode::Command cmd;
            cmd.position = cmd.position =
                std::numeric_limits<double>::quiet_NaN();
            cmd.velocity = 0.0;
            c.SetPositionWaitComplete(cmd, 0.01);
          }
          return;
        }
      } else {
        // Command failed - implement retry logic for frame drops
        auto res = controllers_[i].SetPosition(cmd);
        uint8_t r = 0;

        // Retry up to 3 times for this command
        while (res || (r < 3)) {
          printf("RETRY SEND TO MOTOR %ld\n", i);
          res = controllers_[i].SetPosition(cmd);
          r++;
        }

        // If all retries failed, increment frame drop counter
        if (!res) {
          ++frame_drop_counter[i];
          printf("FRAME DROP: %d frames lost on Motor %ld\n",
                 frame_drop_counter[i], i);
        }

        // Emergency stop if too many consecutive frame drops
        if (frame_drop_counter[i] > 5) {
          // Stop all motors and exit
          for (auto &c : controllers_) {
            c.SetStop();
            moteus::PositionMode::Command cmd;
            cmd.position = cmd.position =
                std::numeric_limits<double>::quiet_NaN();
            cmd.velocity = 0.0;
            c.SetPositionWaitComplete(cmd, 0.01);
          }
          return;
        }
      }
    }

    // Step 2: Check if all motors have completed their trajectories
    bool all_done = true;
    for (auto &st : motor_statuses_) {
      // Check if motor is in position mode and trajectory is complete
      if (!(st.mode == moteus::Mode::kPosition && st.trajectory_complete)) {
        all_done = false;
        break;
      }
    }

    // Update trajectory completion flag if all motors are done
    if (all_done)
      trajectory_complete_.store(true);

    // Step 3: Check for shutdown signal or new commands
    // Note: We do not go back to waiting on command_cv_ after initialization
    lock.lock();

    // Check if we should stop (destructor called)
    if (stop_thread_) {
      lock.unlock();
      return;
    }

    // Check if new command has been enqueued
    if (has_new_command_) {
      // Update targets with new command parameters
      positions = next_positions_;       // Copy new position targets
      accel = next_accel_;               // Copy new acceleration limit
      has_new_command_ = false;          // Reset command flag
      trajectory_complete_.store(false); // Reset completion flag

      // Immediately resend position commands to refresh motor status
      // This ensures trajectory flags are updated with new targets
      for (size_t i = 0; i < MOTORS; ++i) {
        moteus::PositionMode::Command cmd;

        // Handle NaN position values
        if (std::isnan(positions[i])) {
          cmd.position = std::numeric_limits<double>::quiet_NaN();
          cmd.velocity = 0.0;
        } else {
          // Convert radians to revolutions and apply limits
          cmd.position = rad2rev(positions[i]);
          cmd.accel_limit = accel;
          if (!std::isnan(MAX_TORQUE[i])) {
            cmd.maximum_torque = MAX_TORQUE[i];
          }
        }

        // Send command and update status
        auto maybe = controllers_[i].SetPosition(cmd);
        if (maybe)
          motor_statuses_[i] = maybe->values;
      }
    }
    lock.unlock();

    // Step 4: Sleep for ~10ms to maintain ~100Hz control rate
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

/**
 * @brief Enqueues a new simultaneous move for all motors.
 * @param positions Array of target positions in radians for each motor
 * @param accel_limit Acceleration limit applied to all motors (default: 1.0)
 *
 * @details
 * This function implements the producer side of the producer-consumer pattern.
 * It enqueues new position targets for the worker thread to execute.
 *
 * The function is designed to be:
 * - Thread-safe: Uses mutex to protect shared command state
 * - Non-blocking: Returns immediately after enqueueing
 * - Efficient: Minimal locking time
 *
 * Execution Flow:
 * 1. Acquire mutex lock to protect shared command state
 * 2. Copy new position targets and acceleration limit
 * 3. Set has_new_command_ flag to signal worker thread
 * 4. Reset trajectory_complete_ flag to false
 * 5. Release mutex lock
 * 6. Notify worker thread via condition variable
 * 7. Return immediately (non-blocking)
 *
 * The worker thread will pick up these new targets in its next iteration
 * and begin executing the new trajectory.
 *
 * @note This function is thread-safe and can be called from any thread
 * @note NaN values in positions array will disable position control for that
 * motor
 * @note The function returns immediately; use getTrajectoryComplete() to check
 * completion
 * @note The worker thread runs at ~100Hz, so new commands are picked up quickly
 */
void MotorController::SetMotorPositions(
    const std::array<double, MOTORS> &positions, double accel_limit) {
  {
    // Debug output (commented out in production)
    // printf("New position received = [%.3f, %.3f]\n", positions[0],
    // positions[1]);

    // Acquire mutex lock to protect shared command state
    std::lock_guard<std::mutex> lock(command_mutex_);

    // Update command parameters with new targets
    next_positions_ = positions;       // Copy new position targets
    next_accel_ = accel_limit;         // Copy new acceleration limit
    has_new_command_ = true;           // Signal that new command is available
    trajectory_complete_.store(false); // Reset completion flag
  }

  // Notify worker thread that new command is available
  // This wakes up the worker thread if it's waiting for commands
  command_cv_.notify_one();
}
