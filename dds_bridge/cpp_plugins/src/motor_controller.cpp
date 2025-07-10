#include "../include/motor_controller.h"
#include <iostream>
#include <chrono>
#include <cmath> // for std::round

namespace
{
    /// Mathematical constant π.
    constexpr double PI = 3.141592653589793;
    /**
     * @brief Converts radians to revolutions.
     * @param r Angle in radians.
     * @return Equivalent revolutions.
     */
    static inline double rad2rev(double r) { return r / (2 * PI); }

    /**
     * @brief Checks if two values are within a given tolerance.
     * @param a First value.
     * @param b Second value.
     * @param tol Allowed tolerance.
     * @return True if |a - b| ≤ tol.
     */
    static inline bool close_enough(double a, double b, double tol = 0.005)
    {
        return std::fabs(a - b) <= tol;
    }
}

using namespace mjbots; ///< Bring moteus Controller types into scope.

////////////////////////////////////////////////////////////////////////////////
// Constructors / Destructor
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default constructs MotorController with NaN torque limits.
 *
 * Initializes each Moteus controller with default options, then stops
 * and queries each to ensure clean state.
 */
MotorController::MotorController()
    : MAX_TORQUE(init_MAX_TORQUE())
{

    controllers_.reserve(MOTORS);
    for (int i = 0; i < MOTORS; ++i)
    {
        controllers_.emplace_back(createOptions(i));
        motor_statuses_[i] = controllers_[i].SetStop()->values;
    }
    worker_thread_ = std::thread(&MotorController::WorkerLoop, this);
}

/**
 * @brief Constructs MotorController with explicit torque limits.
 * @param max_torque Array specifying maximum torque per motor.
 */
MotorController::MotorController(const std::array<double, MOTORS> &max_torque)
    : MAX_TORQUE{max_torque}
{

    controllers_.reserve(MOTORS);
    for (int i = 0; i < MOTORS; ++i)
    {
        controllers_.emplace_back(createOptions(i));
        motor_statuses_[i] = controllers_[i].SetStop()->values;
    }
    worker_thread_ = std::thread(&MotorController::WorkerLoop, this);
}

/**
 * @brief Destructor stops any running hold threads and cleans up.
 */
MotorController::~MotorController()
{
    { // tell the worker to stop
        std::lock_guard<std::mutex> lock(command_mutex_);
        stop_thread_ = true;
        command_cv_.notify_one();
    }
    worker_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Private Helpers
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Public APIs
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Moves a single motor to a specified position, retrying on communication
 *        failures, and waits until the target is reached or timeout expires.
 * @param pos Desired angular position in radians.
 * @param accel_limit Maximum acceleration (revs/sec²).
 * @param id Motor index [0…MOTORS-1].
 * @throws std::out_of_range if id is invalid.
 * @throws std::runtime_error on timeout or repeated no-response.
 */
void MotorController::SetMotorPosition(double pos,
                                       double accel_limit,
                                       unsigned id)
{
    // 1) Validate motor index
    if (id < 0 || id >= MOTORS)
    {
        throw std::out_of_range("SetPosition: motor ID " +
                                std::to_string(id) +
                                " out of range [0.." +
                                std::to_string(MOTORS - 1) + "]");
    }

    controllers_[id].SetStop();
    // 2) Convert radian position to revolutions (controller expects revs)

    // 3) Build the position command
    moteus::PositionMode::Command cmd;
    cmd.position = rad2rev(pos);   // target in revs
    cmd.accel_limit = accel_limit; // in revs/s^2
    if (!std::isnan(MAX_TORQUE[id]))
        cmd.maximum_torque = MAX_TORQUE[id];
    // Note: stop_position omitted as requested

    // 4) Prepare retry and timeout parameters
    constexpr int kMaxMisses = 2;                                               // max consecutive no-response
    int miss_count = 0;                                                         // how many misses so far
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5); // overall timeout

    // 5) Main poll loop: send command, wait for completion, check status
    while (true)
    {
        // 5a) Check overall timeout
        if (std::chrono::steady_clock::now() > deadline)
        {
            throw std::runtime_error("SetPosition timeout for motor " +
                                     std::to_string(id));
        }

        // 5b) Send command and wait up to 10ms for a reply
        auto maybe = controllers_[id].SetPositionWaitComplete(cmd, 0.01);

        // 5c) Handle no reply case
        if (!maybe)
        {
            // Count up, and if too many misses, give up
            if (++miss_count > kMaxMisses)
            {
                throw std::runtime_error("Motor " +
                                         std::to_string(id) +
                                         " unresponsive after " +
                                         std::to_string(kMaxMisses) +
                                         " tries");
            }
            // Brief pause before retrying to avoid bus flooding
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        // Reset miss counter on successful reply
        miss_count = 0;

        // 5d) Unpack results
        const auto &r = maybe->values;
        // std::cout << "Motor " << id << " has position " << r.position << " revs" << "\n";
        // 5e) Handle fault or timeout modes immediately
        if (r.mode == moteus::Mode::kFault ||
            r.mode == moteus::Mode::kPositionTimeout)
        {
            throw std::runtime_error("Motor " +
                                     std::to_string(id) +
                                     " fault (mode=" +
                                     std::to_string(int(r.mode)) +
                                     ", fault=" +
                                     std::to_string(r.fault) + ")");
        }

        // 5f) Check if we've reached the target (to 2 decimal places)
        if (close_enough(r.position, cmd.position))
        {
            // Success! Exit the loop and return to caller
            break;
        }

        // 5g) Not yet at target: sleep briefly before next poll
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 6) All done — motor reached target safely
}

void MotorController::WorkerLoop()
{
    std::unique_lock<std::mutex> lock(command_mutex_);
    while (true)
    {
        command_cv_.wait(lock, [&]
                         { return has_new_command_ || stop_thread_; });
        if (stop_thread_)
            break;

        auto positions = next_positions_;
        auto accel = next_accel_;
        has_new_command_ = false;
        lock.unlock();

        DoSetMotorPositions_(positions, accel);

        lock.lock();
    }
}

void MotorController::DoSetMotorPositions_(
    const std::array<double, MOTORS>& positions,
    double accel_limit)
{
  // 1) Safety stop & clear any lingering faults
  for (size_t i = 0; i < MOTORS; ++i) {
    controllers_[i].SetStop();
  }

  // 2) Build the position commands once
  moteus::PositionMode::Command cmd[MOTORS];
  for (size_t i = 0; i < MOTORS; ++i) {
    cmd[i].position    = rad2rev(positions[i]);
    cmd[i].accel_limit = accel_limit;
    if (!std::isnan(MAX_TORQUE[i])) {
      cmd[i].maximum_torque = MAX_TORQUE[i];
    }
  }

  // 3) Hammer-and-poll loop, never exits on success (only on timeout or new command)
  while (true) {
    // ——— 3a) Preempt on a brand-new command enqueued by SetMotorPositions() ———
    {
      std::lock_guard<std::mutex> lk(command_mutex_);
      if (has_new_command_) {
        // Return to WorkerLoop(), which will pick up the new positions
        return;
      }
    }

    bool all_done = true;
    for (size_t i = 0; i < MOTORS; ++i) {
      // 3b) Send the hammering command
      auto maybe = controllers_[i].SetPosition(cmd[i]);

      // 3c) No reply? we’ll retry next iteration
      if (!maybe) {
        all_done = false;
        continue;
      }
      auto &st = maybe->values;
      motor_statuses_[i] = st;

      // 3d) Timeout → stop everything and bail out
      if (st.mode == moteus::Mode::kPositionTimeout) {
        for (size_t j = 0; j < MOTORS; ++j) {
          controllers_[j].SetStop();
        }
        trajectory_complete_.store(false);
        return;
      }

      // 3e) Fault → clear & continue hammering
      if (st.fault) {
        controllers_[i].SetStop();
        all_done = false;
        continue;
      }

      // 3f) Only consider “done” if we’re actually in Position mode *and* trajectory_complete
      if (!(st.mode == moteus::Mode::kPosition && st.trajectory_complete)) {
        all_done = false;
      }
    }

    // 3g) If everyone is really done, set the flag (but keep hammering until new command)
    if (all_done) {
      trajectory_complete_.store(true);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MotorController::SetMotorPositions(
    const std::array<double, MOTORS> &positions,
    double accel_limit)
{
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        next_positions_ = positions;
        next_accel_ = accel_limit;
        has_new_command_ = true;
        trajectory_complete_.store(false);
    }
    command_cv_.notify_one();
}
