#include "../include/ik_model.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cmath>

namespace
{
    /// Mathematical constant π.
    constexpr double PI = 3.141592653589793;
    
    /**
     * @brief Check if a file exists
     * @param filename Path to the file
     * @return True if file exists, false otherwise
     */
    bool fileExists(const std::string& filename)
    {
        std::ifstream file(filename);
        return file.good();
    }
}

////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
////////////////////////////////////////////////////////////////////////////////

IKModel::IKModel(const std::string& urdf_path)
{
    // Check if URDF file exists
    if (!fileExists(urdf_path))
    {
        throw std::runtime_error("URDF file not found: " + urdf_path);
    }
    
    initializeModel(urdf_path);
}

////////////////////////////////////////////////////////////////////////////////
// Model Information
////////////////////////////////////////////////////////////////////////////////

int IKModel::getNumJoints() const
{
    return model_->njoints;
}

int IKModel::getNumDOFs() const
{
    return model_->nq;
}

Eigen::MatrixXd IKModel::getJointLimits() const
{
    return joint_limits_;
}

int IKModel::getEndEffectorFrameId() const
{
    return end_effector_frame_id_;
}

////////////////////////////////////////////////////////////////////////////////
// Forward Kinematics
////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4d IKModel::computeForwardKinematics(const Eigen::VectorXd& q)
{
    // Validate input dimensions
    if (q.size() != model_->nq)
    {
        throw std::invalid_argument("Joint angles vector size mismatch. Expected: " + 
                                   std::to_string(model_->nq) + ", Got: " + std::to_string(q.size()));
    }
    
    // Update Pinocchio data with current configuration
    pinocchio::forwardKinematics(*model_, *data_, q);
    pinocchio::updateFramePlacements(*model_, *data_);
    
    // Get end effector transformation
    return data_->oMf[end_effector_frame_id_].toHomogeneousMatrix();
}

void IKModel::computeForwardKinematics(const Eigen::VectorXd& q, 
                                       Eigen::Vector3d& position, 
                                       Eigen::Vector4d& orientation)
{
    // Compute full transformation matrix
    Eigen::Matrix4d transform = computeForwardKinematics(q);
    
    // Extract position
    position = transform.block<3, 1>(0, 3);
    
    // Extract rotation matrix and convert to quaternion
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    orientation = rotationMatrixToQuaternion(rotation);
}

Eigen::MatrixXd IKModel::computeJacobian(const Eigen::VectorXd& q)
{
    // Validate input dimensions
    if (q.size() != model_->nq)
    {
        throw std::invalid_argument("Joint angles vector size mismatch. Expected: " + 
                                   std::to_string(model_->nq) + ", Got: " + std::to_string(q.size()));
    }
    
    // Update Pinocchio data
    pinocchio::computeJointJacobians(*model_, *data_, q);
    pinocchio::updateFramePlacements(*model_, *data_);
    
    // Compute frame Jacobian
    Eigen::MatrixXd J(6, model_->nv);
    pinocchio::computeFrameJacobian(*model_, *data_, q, end_effector_frame_id_, 
                                   pinocchio::ReferenceFrame::WORLD, J);
    
    return J;
}

////////////////////////////////////////////////////////////////////////////////
// Collision Checking
////////////////////////////////////////////////////////////////////////////////

bool IKModel::isCollisionFree(const Eigen::VectorXd& q)
{
    return getCollisionDistance(q) > 0.0;
}

double IKModel::getCollisionDistance(const Eigen::VectorXd& q)
{
    // TODO: Implement collision checking using Pinocchio's collision detection
    // This is a placeholder implementation
    // In a real implementation, you would:
    // 1. Update the robot configuration
    // 2. Check for self-collisions and environment collisions
    // 3. Return the minimum distance to obstacles
    
    // For now, return a positive value (no collision)
    return 1.0;
}

Eigen::VectorXd IKModel::getCollisionGradient(const Eigen::VectorXd& q)
{
    // TODO: Implement collision gradient computation
    // This is a placeholder implementation
    // In a real implementation, you would compute the gradient of the collision distance
    // with respect to joint angles using finite differences or analytical methods
    
    return Eigen::VectorXd::Zero(model_->nv);
}

////////////////////////////////////////////////////////////////////////////////
// Optimization Functions for IK
////////////////////////////////////////////////////////////////////////////////

double IKModel::computeCost(const Eigen::VectorXd& q,
                            const Eigen::Vector3d& target_position,
                            const Eigen::Vector4d& target_orientation,
                            double weight_position,
                            double weight_orientation,
                            double weight_collision)
{
    // Compute current end effector pose
    Eigen::Vector3d current_position;
    Eigen::Vector4d current_orientation;
    computeForwardKinematics(q, current_position, current_orientation);
    
    // Compute individual cost components
    double position_cost = computePositionErrorCost(current_position, target_position);
    double orientation_cost = computeOrientationErrorCost(current_orientation, target_orientation);
    double collision_cost = computeCollisionPenaltyCost(q);
    
    // Return weighted sum
    return weight_position * position_cost + 
           weight_orientation * orientation_cost + 
           weight_collision * collision_cost;
}

Eigen::VectorXd IKModel::computeGradient(const Eigen::VectorXd& q,
                                         const Eigen::Vector3d& target_position,
                                         const Eigen::Vector4d& target_orientation,
                                         double weight_position,
                                         double weight_orientation,
                                         double weight_collision)
{
    // Compute individual gradient components
    Eigen::VectorXd position_gradient = computePositionErrorGradient(q, target_position);
    Eigen::VectorXd orientation_gradient = computeOrientationErrorGradient(q, target_orientation);
    Eigen::VectorXd collision_gradient = getCollisionGradient(q);
    
    // Return weighted sum
    return weight_position * position_gradient + 
           weight_orientation * orientation_gradient + 
           weight_collision * collision_gradient;
}

Eigen::MatrixXd IKModel::computeHessian(const Eigen::VectorXd& q,
                                        const Eigen::Vector3d& target_position,
                                        const Eigen::Vector4d& target_orientation,
                                        double weight_position,
                                        double weight_orientation,
                                        double weight_collision)
{
    // TODO: Implement Hessian computation
    // This is a placeholder implementation
    // In a real implementation, you would compute the second derivatives of the cost function
    // with respect to joint angles
    
    int n = model_->nv;
    return Eigen::MatrixXd::Identity(n, n);
}

////////////////////////////////////////////////////////////////////////////////
// Utility Functions
////////////////////////////////////////////////////////////////////////////////

bool IKModel::isWithinJointLimits(const Eigen::VectorXd& q) const
{
    if (q.size() != joint_limits_.rows())
    {
        return false;
    }
    
    for (int i = 0; i < q.size(); ++i)
    {
        if (q(i) < joint_limits_(i, 0) || q(i) > joint_limits_(i, 1))
        {
            return false;
        }
    }
    return true;
}

void IKModel::clampToJointLimits(Eigen::VectorXd& q) const
{
    if (q.size() != joint_limits_.rows())
    {
        throw std::invalid_argument("Joint angles vector size mismatch");
    }
    
    for (int i = 0; i < q.size(); ++i)
    {
        q(i) = std::max(joint_limits_(i, 0), std::min(joint_limits_(i, 1), q(i)));
    }
}

Eigen::Matrix3d IKModel::quaternionToRotationMatrix(const Eigen::Vector4d& quat)
{
    // Normalize quaternion
    Eigen::Vector4d q = quat.normalized();
    
    // Extract components
    double x = q(0), y = q(1), z = q(2), w = q(3);
    
    // Convert to rotation matrix
    Eigen::Matrix3d R;
    R << 1 - 2*y*y - 2*z*z,  2*x*y - 2*w*z,      2*x*z + 2*w*y,
         2*x*y + 2*w*z,      1 - 2*x*x - 2*z*z,  2*y*z - 2*w*x,
         2*x*z - 2*w*y,      2*y*z + 2*w*x,      1 - 2*x*x - 2*y*y;
    
    return R;
}

Eigen::Vector4d IKModel::rotationMatrixToQuaternion(const Eigen::Matrix3d& rot)
{
    // TODO: Implement robust rotation matrix to quaternion conversion
    // This is a placeholder implementation
    // In a real implementation, you would use a robust method like:
    // - Eigen's built-in conversion
    // - Shepperd's method
    // - Markley's method
    
    // For now, return identity quaternion
    return Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
}

////////////////////////////////////////////////////////////////////////////////
// Private Helper Functions
////////////////////////////////////////////////////////////////////////////////

void IKModel::initializeModel(const std::string& urdf_path)
{
    try
    {
        // Load model from URDF
        model_ = std::make_unique<pinocchio::Model>();
        pinocchio::urdf::buildModel(urdf_path, *model_);
        
        // Create data object
        data_ = std::make_unique<pinocchio::Data>(*model_);
        
        // Find end effector frame
        end_effector_frame_id_ = model_->getFrameId(end_effector_frame_);
        if (end_effector_frame_id_ == model_->frames.size())
        {
            throw std::runtime_error("End effector frame '" + end_effector_frame_ + "' not found in URDF");
        }
        
        // Initialize joint limits
        joint_limits_ = Eigen::MatrixXd(model_->nq, 2);
        for (int i = 0; i < model_->nq; ++i)
        {
            joint_limits_(i, 0) = model_->lowerPositionLimit(i);
            joint_limits_(i, 1) = model_->upperPositionLimit(i);
        }
        
        std::cout << "IKModel initialized successfully:" << std::endl;
        std::cout << "  - Number of joints: " << model_->njoints << std::endl;
        std::cout << "  - Number of DOFs: " << model_->nq << std::endl;
        std::cout << "  - End effector frame: " << end_effector_frame_ << " (ID: " << end_effector_frame_id_ << ")" << std::endl;
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Failed to initialize IKModel: " + std::string(e.what()));
    }
}

double IKModel::computePositionErrorCost(const Eigen::Vector3d& current_pos,
                                         const Eigen::Vector3d& target_pos)
{
    // Compute squared Euclidean distance
    Eigen::Vector3d error = current_pos - target_pos;
    return 0.5 * error.squaredNorm();
}

double IKModel::computeOrientationErrorCost(const Eigen::Vector4d& current_quat,
                                            const Eigen::Vector4d& target_quat)
{
    // Compute quaternion distance (1 - |dot product|)
    // This gives a cost of 0 when orientations match and 2 when they're opposite
    double dot_product = std::abs(current_quat.dot(target_quat));
    return 1.0 - dot_product;
}

double IKModel::computeCollisionPenaltyCost(const Eigen::VectorXd& q)
{
    // TODO: Implement collision penalty cost
    // This is a placeholder implementation
    // In a real implementation, you would:
    // 1. Compute collision distance
    // 2. Apply a penalty function (e.g., barrier function, exponential penalty)
    
    double distance = getCollisionDistance(q);
    if (distance > 0.0)
    {
        // No collision penalty
        return 0.0;
    }
    else
    {
        // Collision penalty (exponential barrier)
        return std::exp(-distance) - 1.0;
    }
}

Eigen::VectorXd IKModel::computePositionErrorGradient(const Eigen::VectorXd& q,
                                                      const Eigen::Vector3d& target_pos)
{
    // Compute current position
    Eigen::Vector3d current_pos;
    Eigen::Vector4d dummy_orientation;
    computeForwardKinematics(q, current_pos, dummy_orientation);
    
    // Compute position error
    Eigen::Vector3d error = current_pos - target_pos;
    
    // Compute Jacobian
    Eigen::MatrixXd J = computeJacobian(q);
    
    // Return gradient: J^T * error
    return J.block(0, 0, 3, J.cols()).transpose() * error;
}

Eigen::VectorXd IKModel::computeOrientationErrorGradient(const Eigen::VectorXd& q,
                                                         const Eigen::Vector4d& target_quat)
{
    // TODO: Implement orientation error gradient
    // This is a placeholder implementation
    // In a real implementation, you would:
    // 1. Compute current orientation
    // 2. Compute orientation error gradient
    // 3. Use the angular part of the Jacobian
    
    return Eigen::VectorXd::Zero(model_->nv);
}
