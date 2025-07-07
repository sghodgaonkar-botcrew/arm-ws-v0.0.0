#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include "ik_model.h"
#include <ceres/ceres.h>
#include <memory>

/**
 * @brief Ceres cost function for IK optimization
 */
class IKCostFunction : public ceres::FirstOrderFunction {
  public:
    /**
     * @brief Constructor
     * @param ik_model Pointer to the IKModel instance
     * @param target_pose Target pose for optimization
     */
    IKCostFunction(std::shared_ptr<IKModel> ik_model,
                   const pinocchio::SE3 &target_pose);

    /**
     * @brief Evaluate the cost function
     * @param parameters Joint configuration (6D vector)
     * @param cost Output cost value
     * @param gradient Output gradient (6D vector)
     * @return true if evaluation was successful
     */
    bool Evaluate(const double *parameters, double *cost,
                  double *gradient) const override;

    /**
     * @brief Get the number of parameters (joints)
     * @return Number of parameters
     */
    int NumParameters() const override { return 6; }

  private:
    std::shared_ptr<IKModel> ik_model_; ///< Pointer to IKModel
    pinocchio::SE3 target_pose_;        ///< Target pose for optimization
};

/**
 * @brief Ceres-based IK solver
 */
class IKSolverCeres {
  public:
    /**
     * @brief Constructor
     * @param ik_model Pointer to the IKModel instance
     */
    explicit IKSolverCeres(std::shared_ptr<IKModel> ik_model);

    /**
     * @brief Destructor
     */
    ~IKSolverCeres() = default;

    /**
     * @brief Solve the inverse kinematics problem
     * @param target_pose Target pose in SE3 format
     * @param initial_guess Initial joint configuration (optional, uses current
     * if not provided)
     * @return Joint configuration that minimizes the cost function
     */
    JointConfig solve(const pinocchio::SE3 &target_pose,
                      const JointConfig &initial_guess = JointConfig());

    /**
     * @brief Set solver options
     * @param max_iterations Maximum number of iterations
     * @param function_tolerance Function tolerance for convergence
     * @param gradient_tolerance Gradient tolerance for convergence
     */
    void setSolverOptions(int max_iterations = 1000,
                          double function_tolerance = 0.0,
                          double gradient_tolerance = 1e-10,
                          double parameter_tolerance = 1e-12);

  private:
    std::shared_ptr<IKModel> ik_model_; ///< Pointer to IKModel
    ceres::GradientProblemSolver::Options solver_options_; ///< Solver options
};

#endif // IK_SOLVER_CERES_H