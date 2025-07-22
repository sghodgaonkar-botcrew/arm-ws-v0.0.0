#include "ik_model.h"
#include <Eigen/Dense>
#include <cstdint>
#include <iostream>
#include <proxsuite/proxqp/dense/dense.hpp>

// Profiling includes
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <sys/resource.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include <vector>

/**
 * @class PerformanceMonitor
 * @brief A class for monitoring and profiling the performance of code blocks.
 *
 * This class provides utilities to track execution time, memory usage, CPU
 * usage, and other metrics for iterative processes. It can generate a summary
 * report and save detailed logs to a CSV file.
 */
class PerformanceMonitor {
  private:
    /** @brief The time point when the monitoring started. */
    std::chrono::high_resolution_clock::time_point start_time_;
    /** @brief A vector to store the duration of each recorded iteration in
     * milliseconds. */
    std::vector<double> iteration_times_;
    /** @brief A vector to store the error value at each recorded iteration. */
    std::vector<double> error_history_;
    /** @brief A vector to store the memory usage (in MB) at each recorded
     * iteration. */
    std::vector<double> memory_usage_;

    // CPU usage tracking
    /** @brief The last recorded CPU time (user time) in microseconds. */
    long last_cpu_time_;
    /** @brief The last recorded wall clock time in microseconds. */
    long last_wall_time_;

    // Thread count tracking
    /** @brief The maximum number of threads observed during monitoring. */
    int max_threads_used_;

  public:
    /**
     * @brief Construct a new Performance Monitor object.
     *
     * Initializes the start time and captures the initial CPU and wall time for
     * subsequent CPU usage calculations.
     */
    PerformanceMonitor() : max_threads_used_(0) {
        start_time_ = std::chrono::high_resolution_clock::now();

        // Initialize CPU tracking
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        last_cpu_time_ =
            usage.ru_utime.tv_sec * 1000000 + usage.ru_utime.tv_usec;

        auto now = std::chrono::high_resolution_clock::now();
        last_wall_time_ = std::chrono::duration_cast<std::chrono::microseconds>(
                              now.time_since_epoch())
                              .count();
    }

    /**
     * @brief Records the performance metrics for a single iteration.
     *
     * @param error The error value for the current iteration.
     * @param iteration_time The execution time of the current iteration in
     * milliseconds.
     *
     * This function captures the iteration time, error, current memory usage,
     * and updates the maximum number of threads used.
     */
    void record_iteration(double error, double iteration_time) {
        iteration_times_.push_back(iteration_time);
        error_history_.push_back(error);

        // Update max threads
        int current_threads = std::thread::hardware_concurrency();
        max_threads_used_ = std::max(max_threads_used_, current_threads);

        // Record memory usage
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        memory_usage_.push_back(usage.ru_maxrss / 1024.0); // Convert to MB
    }

    /**
     * @brief Calculates the CPU usage since the last call to this function.
     *
     * @return The CPU usage as a percentage.
     *
     * This function computes the percentage of CPU time used by the process
     * relative to the elapsed wall clock time.
     */
    double get_cpu_usage() {
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);

        long current_cpu_time =
            usage.ru_utime.tv_sec * 1000000 + usage.ru_utime.tv_usec;
        auto now = std::chrono::high_resolution_clock::now();
        long current_wall_time =
            std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch())
                .count();

        long cpu_diff = current_cpu_time - last_cpu_time_;
        long wall_diff = current_wall_time - last_wall_time_;

        last_cpu_time_ = current_cpu_time;
        last_wall_time_ = current_wall_time;

        return (wall_diff > 0) ? (cpu_diff * 100.0 / wall_diff) : 0.0;
    }

    /**
     * @brief Prints a formatted summary of the performance metrics.
     *
     * The summary includes total execution time, iteration statistics (count,
     * average, min, max), thread and CPU information, memory usage statistics,
     * and convergence details.
     */
    void print_summary() {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto total_duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                  start_time_);

        std::cout << "
" << std::string(60, '=') << std::endl;
        std::cout << "PERFORMANCE SUMMARY" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        // Timing statistics
        std::cout << "Total Execution Time: " << total_duration.count() << " ms"
                  << std::endl;
        std::cout << "Number of Iterations: " << iteration_times_.size()
                  << std::endl;

        if (!iteration_times_.empty()) {
            double avg_iteration_time =
                std::accumulate(iteration_times_.begin(),
                                iteration_times_.end(), 0.0) /
                iteration_times_.size();
            double min_iteration_time = *std::min_element(
                iteration_times_.begin(), iteration_times_.end());
            double max_iteration_time = *std::max_element(
                iteration_times_.begin(), iteration_times_.end());

            std::cout << "Average Iteration Time: " << std::fixed
                      << std::setprecision(3) << avg_iteration_time << " ms"
                      << std::endl;
            std::cout << "Min Iteration Time: " << std::fixed
                      << std::setprecision(3) << min_iteration_time << " ms"
                      << std::endl;
            std::cout << "Max Iteration Time: " << std::fixed
                      << std::setprecision(3) << max_iteration_time << " ms"
                      << std::endl;
        }

        // Thread and CPU information
        std::cout << "Hardware Concurrency: "
                  << std::thread::hardware_concurrency() << " threads"
                  << std::endl;
        std::cout << "Max Threads Used: " << max_threads_used_ << " threads"
                  << std::endl;

        // Memory statistics
        if (!memory_usage_.empty()) {
            double avg_memory = std::accumulate(memory_usage_.begin(),
                                                memory_usage_.end(), 0.0) /
                                memory_usage_.size();
            double max_memory =
                *std::max_element(memory_usage_.begin(), memory_usage_.end());

            std::cout << "Average Memory Usage: " << std::fixed
                      << std::setprecision(2) << avg_memory << " MB"
                      << std::endl;
            std::cout << "Peak Memory Usage: " << std::fixed
                      << std::setprecision(2) << max_memory << " MB"
                      << std::endl;
        }

        // Convergence statistics
        if (!error_history_.empty()) {
            std::cout << "Initial Error: " << std::scientific
                      << std::setprecision(6) << error_history_.front()
                      << std::endl;
            std::cout << "Final Error: " << std::scientific
                      << std::setprecision(6) << error_history_.back()
                      << std::endl;

            // Find convergence rate
            if (error_history_.size() > 1) {
                double convergence_rate =
                    error_history_.back() / error_history_.front();
                std::cout << "Error Reduction Factor: " << std::scientific
                          << std::setprecision(6) << convergence_rate
                          << std::endl;
            }
        }

        std::cout << std::string(60, '=') << std::endl;
    }

    /**
     * @brief Saves the detailed performance log to a CSV file.
     *
     * @param filename The name of the file to save the log to.
     *
     * The CSV file will contain columns for Iteration, Time(ms), Error, and
     * Memory(MB).
     */
    void save_detailed_log(const std::string &filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            file << "Iteration,Time(ms),Error,Memory(MB)
";
            for (size_t i = 0; i < iteration_times_.size(); ++i) {
                file << i << "," << std::fixed << std::setprecision(3)
                     << iteration_times_[i] << "," << std::scientific
                     << std::setprecision(6) << error_history_[i] << ","
                     << std::fixed << std::setprecision(2) << memory_usage_[i]
                     << "
";
            }
            file.close();
            std::cout << "Detailed log saved to: " << filename << std::endl;
        }
    }
};

/**
 * @brief Main entry point for the IK solver application with performance
 * profiling.
 *
 * This function sets up and runs an iterative inverse kinematics (IK) solver
 * for a robotic arm. It uses a Quadratic Programming (QP) approach with
 * backtracking line search to find the joint configurations that achieve a
 * desired end-effector pose.
 *
 * The application is profiled using the PerformanceMonitor class, and the
 * results are printed to the console and saved to a log file. Threading is
 * explicitly limited to a single thread to ensure deterministic performance
 * measurement.
 *
 * @return 0 on successful execution.
 */
int main() {
    // Limit to single thread
    Eigen::setNbThreads(1);

    // Set environment variables to limit threading in other libraries
    setenv("OMP_NUM_THREADS", "1", 1);
    setenv("MKL_NUM_THREADS", "1", 1);
    setenv("OPENBLAS_NUM_THREADS", "1", 1);

    PerformanceMonitor monitor;

    std::cout
        << "Starting IK Solver with Performance Monitoring (Single Thread)..."
        << std::endl;
    std::cout << "Hardware threads available: "
              << std::thread::hardware_concurrency() << std::endl;
    std::cout << "Using threads: " << Eigen::nbThreads() << std::endl;

    // Initialize IK model
    IKModel ik_model(
        "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf",
        "connection_frame", Eigen::Vector<double, 6>::Ones());

    const int nx = ik_model.getNumVelocityVariables();
    const int n_eq = 0;
    const int n_in = 0;

    // Neutral start configuration
    Eigen::VectorXd q = IKModel::NEUTRAL_JOINT_CONFIG;

    // Target pose
    pinocchio::SE3 target_pose = pinocchio::SE3::Identity();
    target_pose.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
    target_pose.rotation() =
        Eigen::Quaterniond(0.7071068, 0, 0.7071068, 0).toRotationMatrix();

    // Preallocate
    pinocchio::SE3 fk_pose;
    Eigen::MatrixXd J(6, nx);
    Eigen::Vector<double, 6> err;
    double error_norm = 0.0;

    // Joint velocity bounds
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(nx, -0.1);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(nx, 0.1);

    // QP solver (box constraints = true)
    proxsuite::proxqp::dense::QP<double> qp(nx, n_eq, n_in, true);

    // Line search parameters
    const double alpha_init = 1.0;
    const double rho = 0.5;
    const double alpha_min = 1e-4;

    std::cout << "Starting optimization loop..." << std::endl;

    for (int iter = 0; iter < 1000; ++iter) {
        auto iteration_start = std::chrono::high_resolution_clock::now();

        // 1) Forward kinematics & Jacobian
        fk_pose = ik_model.computeForwardKinematics(q);
        J = ik_model.computeGeometricJacobian(q); // size 6×nx

        // 2) Compute task error
        err = -(pinocchio::log6(target_pose.inverse() * fk_pose)).toVector();
        error_norm = err.norm();

        if (error_norm < 1e-6) {
            auto iteration_end = std::chrono::high_resolution_clock::now();
            auto iteration_duration =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    iteration_end - iteration_start);
            monitor.record_iteration(error_norm,
                                     iteration_duration.count() / 1000.0);

            std::cout << "
Converged in " << iter << " iterations!"
                      << std::endl;
            std::cout << "Final joint angles (degrees): "
                      << q.transpose() * (180 / M_PI) << std::endl;
            break;
        }

        // 3) Build QP: H = JᵀJ, g = -Jᵀ err
        Eigen::MatrixXd H = J.transpose() * J;
        Eigen::VectorXd g = -J.transpose() * err;

        // 4) Initialize and solve QP with box constraints
        qp.init(H, g, Eigen::MatrixXd(),
                Eigen::VectorXd(), // no equality constraints (A_eq, b_eq)
                Eigen::MatrixXd(),
                Eigen::VectorXd(), // no general inequalities (C, l)
                Eigen::VectorXd(), // no general upper bounds (u)
                lb, ub             // box constraints
        );
        qp.solve();

        Eigen::VectorXd dq = qp.results.x;

        // 5) Backtracking line search along dq
        double alpha = alpha_init;
        const double err0 = error_norm;
        pinocchio::SE3 fk_tmp;
        Eigen::Vector<double, 6> err_tmp;
        double err_tmp_norm;

        while (alpha > alpha_min) {
            Eigen::VectorXd q_test = q + alpha * dq;
            // evaluate new error
            fk_tmp = ik_model.computeForwardKinematics(q_test);
            err_tmp =
                -(pinocchio::log6(target_pose.inverse() * fk_tmp)).toVector();
            err_tmp_norm = err_tmp.norm();
            if (err_tmp_norm < err0)
                break;
            alpha *= rho;
        }

        // 6) Update joint configuration
        q += alpha * dq;

        // Record performance metrics
        auto iteration_end = std::chrono::high_resolution_clock::now();
        auto iteration_duration =
            std::chrono::duration_cast<std::chrono::microseconds>(
                iteration_end - iteration_start);
        monitor.record_iteration(error_norm,
                                 iteration_duration.count() / 1000.0);

        // Progress output
        if (iter % 10 == 0) {
            std::cout << "iter " << std::setw(3) << iter
                      << "  err = " << std::scientific << std::setprecision(6)
                      << error_norm << "  time = " << std::fixed
                      << std::setprecision(3)
                      << (iteration_duration.count() / 1000.0) << "ms"
                      << "" << std::flush;
        }
    }

    // Print final summary
    monitor.print_summary();

    // Save detailed log
    monitor.save_detailed_log("ik_solver_performance_log.csv");

    return 0;
}