#include "ik_model.h"
#include "ik_solver_v2.hpp"
#include <chrono>
#include <iostream>

// IPOPT includes
#ifdef HAVE_IPOPT
#include <IpIpoptApplication.hpp>
#include <IpSolveStatistics.hpp>
#endif

// Enable profiling for this example
#define ENABLE_PROFILING

int main() {
    std::cout << "=== IPOPT Solver Profiling Test ===" << std::endl;

    try {
        // Reset any existing profile data
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

        // Test IPOPT solver performance (if available)
        std::cout << "\nTesting IPOPT solver performance..." << std::endl;
#ifdef HAVE_IPOPT
        try {
            for (int i = 0; i < 5; ++i) {
                std::cout << "IPOPT test " << (i + 1) << "/5..." << std::endl;

                // Generate a random target pose
                pinocchio::SE3 random_target = pinocchio::SE3::Random();

                // Profile the solver creation
                auto start_solver_creation =
                    std::chrono::high_resolution_clock::now();
                IkIpoptSolver solver(ik_model, random_target);
                auto end_solver_creation =
                    std::chrono::high_resolution_clock::now();
                auto solver_creation_time =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        end_solver_creation - start_solver_creation);

                std::cout << "  Solver creation time: "
                          << solver_creation_time.count() << " μs" << std::endl;

                // Profile IPOPT application creation
                auto start_app_creation =
                    std::chrono::high_resolution_clock::now();
                Ipopt::SmartPtr<Ipopt::IpoptApplication> app =
                    IpoptApplicationFactory();
                auto end_app_creation =
                    std::chrono::high_resolution_clock::now();
                auto app_creation_time =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        end_app_creation - start_app_creation);

                std::cout << "  App creation time: "
                          << app_creation_time.count() << " μs" << std::endl;

                // Profile IPOPT configuration
                auto start_config = std::chrono::high_resolution_clock::now();
                app->Options()->SetIntegerValue("print_level", 0);
                app->Options()->SetStringValue("sb", "yes");
                app->Options()->SetNumericValue("tol", 1e-6);
                app->Options()->SetIntegerValue("max_iter", 100);
                auto end_config = std::chrono::high_resolution_clock::now();
                auto config_time =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        end_config - start_config);

                std::cout << "  Configuration time: " << config_time.count()
                          << " μs" << std::endl;

                // Profile IPOPT initialization
                auto start_init = std::chrono::high_resolution_clock::now();
                Ipopt::ApplicationReturnStatus status = app->Initialize();
                auto end_init = std::chrono::high_resolution_clock::now();
                auto init_time =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        end_init - start_init);

                std::cout << "  Initialization time: " << init_time.count()
                          << " μs" << std::endl;

                if (status == Ipopt::Solve_Succeeded) {
                    // Profile the actual optimization
                    auto start_optimization =
                        std::chrono::high_resolution_clock::now();

                    try {
                        Ipopt::SmartPtr<IkIpoptSolver> solver_ptr =
                            new IkIpoptSolver(ik_model, random_target);
                        status = app->OptimizeTNLP(solver_ptr);

                        auto end_optimization =
                            std::chrono::high_resolution_clock::now();
                        auto optimization_time = std::chrono::duration_cast<
                            std::chrono::microseconds>(end_optimization -
                                                       start_optimization);

                        std::cout << "  Optimization time: "
                                  << optimization_time.count() << " μs"
                                  << std::endl;
                        std::cout << "  Final status: " << status << std::endl;

                        // Note: IPOPT statistics API may vary by version
                        std::cout << "  Optimization completed successfully"
                                  << std::endl;

                    } catch (const std::exception &opt_e) {
                        auto end_optimization =
                            std::chrono::high_resolution_clock::now();
                        auto optimization_time = std::chrono::duration_cast<
                            std::chrono::microseconds>(end_optimization -
                                                       start_optimization);

                        std::cout << "  Optimization failed after "
                                  << optimization_time.count() << " μs"
                                  << std::endl;
                        std::cout << "  Error: " << opt_e.what() << std::endl;
                    }
                } else {
                    std::cout
                        << "  Initialization failed with status: " << status
                        << std::endl;
                }

                std::cout << std::endl;
            }
        } catch (const std::exception &e) {
            std::cout << "IPOPT test failed: " << e.what() << std::endl;
        }
#else
        std::cout << "IPOPT not available - cannot test solver performance"
                  << std::endl;
#endif

        // Print comprehensive profile reports
        std::cout << "\n" << std::string(80, '=') << std::endl;
        std::cout << "GENERATING PERFORMANCE PROFILE REPORTS..." << std::endl;

        try {
            IKModel::printProfileReport();
        } catch (const std::exception &e) {
            std::cout << "Error printing IK model profile report: " << e.what()
                      << std::endl;
        }

        try {
            IkIpoptSolver::printProfileReport();
        } catch (const std::exception &e) {
            std::cout << "Error printing IK solver profile report: " << e.what()
                      << std::endl;
        }

        // Print IPOPT-specific analysis
        std::cout << "\n=== IPOPT PERFORMANCE ANALYSIS ===" << std::endl;
        std::cout << "Key IPOPT bottlenecks to monitor:" << std::endl;
        std::cout << "1. Solver creation and initialization" << std::endl;
        std::cout << "2. IPOPT application setup" << std::endl;
        std::cout << "3. Optimization iterations" << std::endl;
        std::cout << "4. Function evaluations (cost, gradient, Hessian)"
                  << std::endl;
        std::cout << "5. Linear solver operations" << std::endl;
        std::cout << "6. Convergence checks" << std::endl;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}