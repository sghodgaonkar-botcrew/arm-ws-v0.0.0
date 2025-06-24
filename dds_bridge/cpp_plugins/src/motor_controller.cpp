#include "../include/motor_controller.h"
#include <iostream>
#include <limits>
#include <cmath> // for std::round

namespace
{
    constexpr double PI = 3.141592653589793;
    static inline double rad2rev(double r) { return r / (2 * PI); }
    static inline bool close_enough(double a, double b, double tol = 0.005)
    {
        return std::fabs(a - b) <= tol;
    }
}

using namespace mjbots;

// MotorController::MotorController()
// {

//     controllers_.reserve(MOTORS);
//     for (int i = 0; i < MOTORS; ++i)
//     {
//         moteus::Controller::Options options;
//         options.id = i;
//         options.position_format.accel_limit = moteus::kFloat;
//         controllers_.emplace_back(options);
//         controllers_[i].SetStop();
//         controllers_[i].SetQuery();
//     }
// }
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

MotorController::~MotorController()
{
    stopHoldThreads();
}


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

/// @brief This function is responsible to set a certain motor position for the motor and hold it at that position, provided that `servo.timeout_mode = 10` in the motor configuration. If it was not so, then the motor will release torque once the position is reached.
/// @param pos The angular position of the motor in radians.
/// @param accel_limit The acceleration limit of the motor in revs/s^2.
/// @param id [0,...,5] The `id.id` of the motor that needs to perform this movement.
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
        std::cout << "Motor " << id << " has position " << r.position << " revs" << "\n";
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

        // 5f) Check if we’ve reached the target (to 2 decimal places)
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

void MotorController::SetArmPose(const double positions[MOTORS],
                                 const double accel_limit)
{
    // 1) Stop any existing hold threads
    stopHoldThreads();
    hold_running_ = true;

    // 3) Launch exactly two worker threads to time‐slice through all motors
    const int num_workers = 2;
    hold_threads_.clear();
    hold_threads_.reserve(num_workers);

    for (int w = 0; w < num_workers; ++w)
    {
        hold_threads_.emplace_back(
            [this, positions, accel_limit, w]()
            {
                while (hold_running_)
                {
                    // Each worker services motors w, w+2, w+4, ...
                    for (int i = w; i < MOTORS; i += num_workers)
                    {

                        moteus::PositionMode::Command cmd;
                        cmd.position = rad2rev(positions[i]);
                        cmd.accel_limit = accel_limit;
                        if (!std::isnan(MAX_TORQUE[i]))
                            cmd.maximum_torque = MAX_TORQUE[i];

                        // Serialize access to each controller
                        {
                            std::lock_guard<std::mutex> lock(controller_mutexes_[i]);
                            controllers_[i].SetPosition(cmd);
                        }

                        // Time‐slice: sleep for 20 ms before next motor
                        std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    }
                }
            });
    }

    // 4) Poll each motor until all have reached their targets
    bool pose_reached[MOTORS] = {}; // all false initially
    constexpr double kTol = 0.005;  // tolerance in revolutions

    while (std::any_of(pose_reached, pose_reached + MOTORS,
                       [](bool done)
                       { return !done; }))
    {
        for (int m = 0; m < MOTORS; ++m)
        {
            if (!pose_reached[m])
            {
                mjbots::moteus::Optional<mjbots::moteus::Controller::Result> r;
                {
                    std::lock_guard<std::mutex> lock(controller_mutexes_[m]);
                    r = controllers_[m].SetQuery();
                }
                if (r)
                {
                    if (close_enough((r->values).position, rad2rev(positions[m])))
                        pose_reached[m] = true;
                    // std::cerr << "Motor " << m << " reached target\n";
                }
            }
        }
        // Throttle polling to avoid bus flooding
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 5) All motors reached targets—clean up threads
    stopHoldThreads();
}

void MotorController::ReleaseMotorTorque(unsigned id)
{
    controllers_[id].SetStop();
}

void MotorController::ReleaseArmTorque()
{
    for (int i = 0; i < MOTORS; i++)
        controllers_[i].SetStop();
}
