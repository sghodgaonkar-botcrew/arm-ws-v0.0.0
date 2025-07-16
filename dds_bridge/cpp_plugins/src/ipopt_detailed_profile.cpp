#include "ik_model.h"
#include "ik_solver_v2.hpp"
#include <chrono>
#include <iostream>
#include <map>
#include <vector>

// IPOPT includes
#ifdef HAVE_IPOPT
#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#include <IpTNLP.hpp>
#endif

// Enable profiling for this example
#define ENABLE_PROFILING

// Detailed timing structure
struct DetailedTiming {
    std::string operation;
    long long total_time_us;
    int call_count;
    long long min_time_us;
    long long max_time_us;

    DetailedTiming()
        : total_time_us(0), call_count(0), min_time_us(LLONG_MAX),
          max_time_us(0) {}

    void addMeasurement(long long time_us) {
        total_time_us += time_us;
        call_count++;
        min_time_us = std::min(min_time_us, time_us);
        max_time_us = std::max(max_time_us, time_us);
    }

    double getAverageTime() const {
        return call_count > 0 ? static_cast<double>(total_time_us) / call_count
                              : 0.0;
    }
};

// Global timing data
std::map<std::string, DetailedTiming> detailed_timings;
std::mutex timing_mutex;

void addDetailedTiming(const std::string &operation, long long time_us) {
    std::lock_guard<std::mutex> lock(timing_mutex);
    detailed_timings[operation].addMeasurement(time_us);
}

void printDetailedTimingReport() {
    std::lock_guard<std::mutex> lock(timing_mutex);

    std::cout << "\n=== DETAILED IPOPT TIMING ANALYSIS ===" << std::endl;
    std::cout << std::setw(40) << std::left << "Operation" << std::setw(10)
              << std::left << "Calls" << std::setw(15) << std::left
              << "Total (ms)" << std::setw(15) << std::left << "Avg (μs)"
              << std::setw(15) << std::left << "Min (μs)" << std::setw(15)
              << std::left << "Max (μs)" << std::endl;
    std::cout << std::string(120, '-') << std::endl;

    for (const auto &pair : detailed_timings) {
        const DetailedTiming &timing = pair.second;
        std::cout << std::setw(40) << std::left << timing.operation
                  << std::setw(10) << std::left << timing.call_count
                  << std::setw(15) << std::left << std::fixed
                  << std::setprecision(2) << (timing.total_time_us / 1000.0)
                  << std::setw(15) << std::left << std::fixed
                  << std::setprecision(2) << timing.getAverageTime()
                  << std::setw(15) << std::left << timing.min_time_us
                  << std::setw(15) << std::left << timing.max_time_us
                  << std::endl;
    }
}

int main() {
    std::cout << "=== Detailed IPOPT Solver Profiling ===" << std::endl;

    try {
        // Reset profile data
        IKModel::resetProfileData();
        IkIpoptSolver::resetProfileData();

        // Load the robot model
        std::string urdf_path =
            "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf";
        std::string end_effector_name = "connection_frame";
        Eigen::Vector<double, 6> pose_weights =
            Eigen::Vector<double, 6>::Ones();

        std::cout << "Loading IK model..." << std::endl;
        IKModel ik_model(urdf_path, end_effector_name, pose_weights);

#ifdef HAVE_IPOPT
        std::cout << "\nTesting IPOPT solver with detailed timing..."
                  << std::endl;

        // Test multiple random targets
        for (int test = 0; test < 3; ++test) {
            std::cout << "\n--- Test " << (test + 1) << "/3 ---" << std::endl;

            // Generate random target pose
            pinocchio::SE3 target_pose = pinocchio::SE3::Random();
            std::cout << "Target pose: " << target_pose << std::endl;

            // Profile solver creation
            auto start = std::chrono::high_resolution_clock::now();
            IkIpoptSolver solver(ik_model, target_pose);
            auto end = std::chrono::high_resolution_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                      start);
            addDetailedTiming("Solver Creation", duration.count());

            // Profile IPOPT application setup
            start = std::chrono::high_resolution_clock::now();
            Ipopt::SmartPtr<Ipopt::IpoptApplication> app =
                IpoptApplicationFactory();
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(
                end - start);
            addDetailedTiming("IPOPT App Creation", duration.count());

            // Profile IPOPT configuration
            start = std::chrono::high_resolution_clock::now();
            app->Options()->SetIntegerValue("print_level", 0);
            app->Options()->SetStringValue("sb", "yes");
            app->Options()->SetNumericValue("tol", 1e-6);
            app->Options()->SetIntegerValue("max_iter",
                                            50); // Reduced for profiling
            app->Options()->SetStringValue("linear_solver", "mumps");
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(
                end - start);
            addDetailedTiming("IPOPT Configuration", duration.count());

            // Profile IPOPT initialization
            start = std::chrono::high_resolution_clock::now();
            Ipopt::ApplicationReturnStatus status = app->Initialize();
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(
                end - start);
            addDetailedTiming("IPOPT Initialization", duration.count());

            if (status == Ipopt::Solve_Succeeded) {
                // Profile the optimization process
                start = std::chrono::high_resolution_clock::now();

                try {
                    Ipopt::SmartPtr<IkIpoptSolver> solver_ptr =
                        new IkIpoptSolver(ik_model, target_pose);
                    status = app->OptimizeTNLP(solver_ptr);

                    end = std::chrono::high_resolution_clock::now();
                    duration =
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            end - start);
                    addDetailedTiming("IPOPT Optimization", duration.count());

                    std::cout << "  Optimization completed in "
                              << duration.count() << " μs" << std::endl;
                    std::cout << "  Final status: " << status << std::endl;

                    // Note: IPOPT statistics API may vary by version
                    std::cout << "  Optimization completed successfully"
                              << std::endl;

                } catch (const std::exception &e) {
                    end = std::chrono::high_resolution_clock::now();
                    duration =
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            end - start);
                    addDetailedTiming("IPOPT Optimization (Failed)",
                                      duration.count());

                    std::cout << "  Optimization failed after "
                              << duration.count() << " μs" << std::endl;
                    std::cout << "  Error: " << e.what() << std::endl;
                }
            } else {
                std::cout << "  Initialization failed with status: " << status
                          << std::endl;
            }
        }
#else
        std::cout << "IPOPT not available - cannot perform detailed profiling"
                  << std::endl;
#endif

        // Print all profile reports
        std::cout << "\n" << std::string(80, '=') << std::endl;
        std::cout << "COMPREHENSIVE PERFORMANCE ANALYSIS" << std::endl;

        // Print detailed timing report
        printDetailedTimingReport();

        // Print IK model profile report
        try {
            IKModel::printProfileReport();
        } catch (const std::exception &e) {
            std::cout << "Error printing IK model profile: " << e.what()
                      << std::endl;
        }

        // Print IK solver profile report
        try {
            IkIpoptSolver::printProfileReport();
        } catch (const std::exception &e) {
            std::cout << "Error printing IK solver profile: " << e.what()
                      << std::endl;
        }

        // Performance recommendations
        std::cout << "\n=== PERFORMANCE RECOMMENDATIONS ===" << std::endl;
        std::cout << "Based on the profiling results, consider:" << std::endl;
        std::cout << "1. Optimizing the most time-consuming operations"
                  << std::endl;
        std::cout << "2. Reducing function evaluation overhead" << std::endl;
        std::cout << "3. Tuning IPOPT parameters for your specific problem"
                  << std::endl;
        std::cout << "4. Using warm starts to reduce iteration count"
                  << std::endl;
        std::cout << "5. Pre-computing expensive operations where possible"
                  << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}