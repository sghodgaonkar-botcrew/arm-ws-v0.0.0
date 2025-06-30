#ifndef IK_MODEL_H
#define IK_MODEL_H

#include <string>
#include <vector>
// #include "Eigen/Dense"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
// #include "pinocchio/algorithm/collision.hpp"
#include <memory>

/**
 * @brief Inverse Kinematics Model class for robotic arm control
 * 
 * This class provides functionality for:
 * - Loading and storing robot model from URDF
 * - Forward kinematics using Pinocchio
 * - Collision checking
 * - Cost function, gradient, and Hessian computation for IK optimization
 */
class IKModel
{
public:
    /**
     * @brief Constructor
     * @param urdf_path Path to the URDF file describing the robot
     * @param end_effector_frame Name of the end effector frame
     */
    explicit IKModel(const std::string& urdf_path);
    
    /**
     * @brief Destructor
     */
    ~IKModel() = default;
    
    // Disable copy constructor and assignment operator
    IKModel(const IKModel&) = delete;
    IKModel& operator=(const IKModel&) = delete;
    
    // Allow move constructor and assignment
    IKModel(IKModel&&) = default;
    IKModel& operator=(IKModel&&) = default;

    ////////////////////////////////////////////////////////////////////////////////
    // Model Information
    ////////////////////////////////////////////////////////////////////////////////
    
    /**
     * @brief Get the number of joints in the robot model
     * @return Number of joints
     */
    int getNumJoints() const;
    
    /**
     * @brief Get the number of degrees of freedom
     * @return Number of DOFs
     */
    int getNumDOFs() const;
    
    /**
     * @brief Get joint limits
     * @return Matrix with lower and upper joint limits [lower, upper]
     */
    Eigen::MatrixXd getJointLimits() const;
    
    /**
     * @brief Get the end effector frame ID
     * @return Frame ID
     */
    int getEndEffectorFrameId() const;

    ////////////////////////////////////////////////////////////////////////////////
    // Forward Kinematics
    ////////////////////////////////////////////////////////////////////////////////
    
    /**
     * @brief Compute forward kinematics for given joint angles
     * @param q Joint angles vector
     * @return 4x4 transformation matrix of end effector pose
     */
    Eigen::Matrix4d computeForwardKinematics(const Eigen::VectorXd& q);
    
    /**
     * @brief Compute forward kinematics and return position and orientation separately
     * @param q Joint angles vector
     * @param position Output position vector (x, y, z)
     * @param orientation Output orientation quaternion (x, y, z, w)
     */
    void computeForwardKinematics(const Eigen::VectorXd& q, 
                                  Eigen::Vector3d& position, 
                                  Eigen::Vector4d& orientation);
    
    /**
     * @brief Compute Jacobian matrix for the end effector
     * @param q Joint angles vector
     * @return 6xN Jacobian matrix (linear and angular velocities)
     */
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& q);

    ////////////////////////////////////////////////////////////////////////////////
    // Collision Checking
    ////////////////////////////////////////////////////////////////////////////////
    
    /**
     * @brief Check if a configuration is collision-free
     * @param q Joint angles vector
     * @return True if collision-free, false otherwise
     */
    bool isCollisionFree(const Eigen::VectorXd& q);
    
    /**
     * @brief Get collision distance (negative if in collision)
     * @param q Joint angles vector
     * @return Minimum distance to obstacles (negative if in collision)
     */
    double getCollisionDistance(const Eigen::VectorXd& q);
    
    /**
     * @brief Get collision gradient (derivative of collision distance w.r.t. joint angles)
     * @param q Joint angles vector
     * @return Gradient vector
     */
    Eigen::VectorXd getCollisionGradient(const Eigen::VectorXd& q);

    ////////////////////////////////////////////////////////////////////////////////
    // Optimization Functions for IK
    ////////////////////////////////////////////////////////////////////////////////
    
    /**
     * @brief Compute cost function for IK optimization
     * @param q Joint angles vector
     * @param target_position Target end effector position
     * @param target_orientation Target end effector orientation (quaternion)
     * @param weight_position Weight for position error
     * @param weight_orientation Weight for orientation error
     * @param weight_collision Weight for collision penalty
     * @return Total cost value
     */
    double computeCost(const Eigen::VectorXd& q,
                       const Eigen::Vector3d& target_position,
                       const Eigen::Vector4d& target_orientation,
                       double weight_position = 1.0,
                       double weight_orientation = 1.0,
                       double weight_collision = 10.0);
    
    /**
     * @brief Compute gradient of cost function
     * @param q Joint angles vector
     * @param target_position Target end effector position
     * @param target_orientation Target end effector orientation (quaternion)
     * @param weight_position Weight for position error
     * @param weight_orientation Weight for orientation error
     * @param weight_collision Weight for collision penalty
     * @return Gradient vector
     */
    Eigen::VectorXd computeGradient(const Eigen::VectorXd& q,
                                    const Eigen::Vector3d& target_position,
                                    const Eigen::Vector4d& target_orientation,
                                    double weight_position = 1.0,
                                    double weight_orientation = 1.0,
                                    double weight_collision = 10.0);
    
    /**
     * @brief Compute Hessian matrix of cost function
     * @param q Joint angles vector
     * @param target_position Target end effector position
     * @param target_orientation Target end effector orientation (quaternion)
     * @param weight_position Weight for position error
     * @param weight_orientation Weight for orientation error
     * @param weight_collision Weight for collision penalty
     * @return Hessian matrix
     */
    Eigen::MatrixXd computeHessian(const Eigen::VectorXd& q,
                                   const Eigen::Vector3d& target_position,
                                   const Eigen::Vector4d& target_orientation,
                                   double weight_position = 1.0,
                                   double weight_orientation = 1.0,
                                   double weight_collision = 10.0);

    ////////////////////////////////////////////////////////////////////////////////
    // Utility Functions
    ////////////////////////////////////////////////////////////////////////////////
    
    /**
     * @brief Check if joint angles are within limits
     * @param q Joint angles vector
     * @return True if within limits, false otherwise
     */
    bool isWithinJointLimits(const Eigen::VectorXd& q) const;
    
    /**
     * @brief Clamp joint angles to limits
     * @param q Joint angles vector (modified in place)
     */
    void clampToJointLimits(Eigen::VectorXd& q) const;
    
    /**
     * @brief Convert quaternion to rotation matrix
     * @param quat Quaternion (x, y, z, w)
     * @return 3x3 rotation matrix
     */
    static Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Vector4d& quat);
    
    /**
     * @brief Convert rotation matrix to quaternion
     * @param rot 3x3 rotation matrix
     * @return Quaternion (x, y, z, w)
     */
    static Eigen::Vector4d rotationMatrixToQuaternion(const Eigen::Matrix3d& rot);

private:
    // Pinocchio model and data
    std::unique_ptr<pinocchio::Model> model_;
    std::unique_ptr<pinocchio::Data> data_;
    
    // End effector information
    std::string end_effector_frame_;
    int end_effector_frame_id_;
    
    // Joint limits
    Eigen::MatrixXd joint_limits_;
    
    ////////////////////////////////////////////////////////////////////////////////
    // Private Helper Functions
    ////////////////////////////////////////////////////////////////////////////////
    
    /**
     * @brief Initialize the robot model from URDF
     * @param urdf_path Path to URDF file
     */
    void initializeModel(const std::string& urdf_path);
    
    /**
     * @brief Compute position error cost
     * @param current_pos Current end effector position
     * @param target_pos Target position
     * @return Position error cost
     */
    double computePositionErrorCost(const Eigen::Vector3d& current_pos,
                                   const Eigen::Vector3d& target_pos);
    
    /**
     * @brief Compute orientation error cost
     * @param current_quat Current end effector orientation
     * @param target_quat Target orientation
     * @return Orientation error cost
     */
    double computeOrientationErrorCost(const Eigen::Vector4d& current_quat,
                                      const Eigen::Vector4d& target_quat);
    
    /**
     * @brief Compute collision penalty cost
     * @param q Joint angles vector
     * @return Collision penalty cost
     */
    double computeCollisionPenaltyCost(const Eigen::VectorXd& q);
    
    /**
     * @brief Compute gradient of position error
     * @param q Joint angles vector
     * @param target_pos Target position
     * @return Position error gradient
     */
    Eigen::VectorXd computePositionErrorGradient(const Eigen::VectorXd& q,
                                                 const Eigen::Vector3d& target_pos);
    
    /**
     * @brief Compute gradient of orientation error
     * @param q Joint angles vector
     * @param target_quat Target orientation
     * @return Orientation error gradient
     */
    Eigen::VectorXd computeOrientationErrorGradient(const Eigen::VectorXd& q,
                                                    const Eigen::Vector4d& target_quat);
};

#endif // IK_MODEL_H
