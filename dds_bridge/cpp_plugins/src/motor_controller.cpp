#include "../include/motor_controller.h"
#include <iostream>
#include <limits>
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
        moteus::Controller::Options options;
        options.id = i;
        options.position_format.accel_limit = moteus::kFloat;
        controllers_.emplace_back(options);
        controllers_[i].SetStop();
        controllers_[i].SetQuery();
    }
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
        moteus::Controller::Options options;
        options.id = i;
        options.position_format.accel_limit = moteus::kFloat;
        controllers_.emplace_back(options);
        controllers_[i].SetStop();
        controllers_[i].SetQuery();
    }
}

/**
 * @brief Destructor stops any running hold threads and cleans up.
 */
MotorController::~MotorController()
{
    stopHoldThreads();
}

////////////////////////////////////////////////////////////////////////////////
// Private Helpers
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Stops and joins all active hold threads if running.
 *
 * Uses a lock_guard to protect against concurrent modifications.
 */
void MotorController::stopHoldThreads()
{
    std::lock_guard<std::mutex> lk(hold_mutex_);
    if (hold_running_)
    {
        hold_running_ = false;
        for (auto &t : hold_threads_)
        {
            if (t.joinable())
                t.join();
        }
        hold_threads_.clear();
    }
}

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
void MotorController::SetPosition(double pos,
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

/**
 * @brief Moves all motors to the specified "arm pose" by launching
 *        worker threads that cycle through each motor command.
 * @param positions Array of target positions (radians) per motor.
 * @param accel_limit Shared acceleration limit for all motors.
 */
void MotorController::SetArmPose(const double positions[MOTORS],
                                 const double accel_limit)
{
    // Step 1: Safety first - release all motor torques to prevent conflicts
    this->ReleaseArmTorque();
    
    // Step 2: Prepare position commands for all motors
    moteus::PositionMode::Command cmd[MOTORS];
    for (int i = 0; i < MOTORS; i++)
    {
        // std::cout << "Setting position for motor with id " << i << std::endl;
        cmd[i].position = rad2rev(positions[i]);  // Convert radians to revolutions
        cmd[i].accel_limit = accel_limit;         // Set acceleration limit
        if (!std::isnan(MAX_TORQUE[i]))
            cmd[i].maximum_torque = MAX_TORQUE[i]; // Apply torque limit if specified
        controllers_[i].SetPosition(cmd[i]);      // Send initial position command
    }
    
    // Step 3: Polling loop to ensure all motors reach their targets
    bool all_done = false;
    while (!all_done)
    {
        all_done = true;  // Assume all motors are done, will be set to false if any aren't
        
        // Step 3a: Check each motor's status
        for (int i = 0; i < MOTORS; ++i) {
            auto maybe = controllers_[i].SetQuery();  // Query motor status

            // Step 3b: Handle communication failures (no reply from motor)
            if (!maybe) {
                all_done = false;                    // Mark as not done
                controllers_[i].SetPosition(cmd[i]); // Re-send position command
                continue;                            // Move to next motor
            }

            // Step 3c: Handle motor faults - clear fault and restart movement
            if (maybe->values.fault) {
                std::cerr 
                  << "Motor " << i << " fault " << maybe->values.fault << "\n";

                controllers_[i].SetStop();            // Clear fault latch by stopping
                controllers_[i].SetPosition(cmd[i]);  // Re-enable movement to target
                all_done = false;                     // Mark as not done
                continue;                             // Move to next motor
            }

            // Step 3d: Check if motor has reached target position
            if (!close_enough(maybe->values.position, cmd[i].position)) {
                controllers_[i].SetPosition(cmd[i]);  // Re-send command if not at target
                all_done = false;                     // Mark as not done
            }
        }

        // Step 3e: Brief pause to avoid overwhelming the communication bus
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/**
 * @brief Immediately stops (releases torque on) a single motor.
 * @param id Motor index.
 */
void MotorController::ReleaseMotorTorque(unsigned id)
{
    controllers_[id].SetStop();
}

/**
 * @brief Immediately stops (releases torque on) all motors.
 */
void MotorController::ReleaseArmTorque()
{
    for (int i = 0; i < MOTORS; i++)
        controllers_[i].SetStop();
}
