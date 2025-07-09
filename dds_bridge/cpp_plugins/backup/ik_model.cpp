#include "ik_model.h"
#include <cstring> // for strncmp
#include <iomanip>
#include <iostream>

// Define static member
const JointConfig IKModel::NEUTRAL_JOINT_CONFIG = JointConfig::Zero();

IKModel::IKModel(const std::string &urdf_path,
                 const std::string &end_effector_name,
                 const Eigen::Vector<double, 6> &pose_weights)
    : urdf_path_(urdf_path), end_effector_name_(end_effector_name),
      current_joint_config_(NEUTRAL_JOINT_CONFIG),
      pose_weights_(pose_weights.asDiagonal()),
      joint_limits_initialized_(false) {
    try {
        // Load the URDF model
        pinocchio::urdf::buildModel(urdf_path, model_);

        // Create the data object
        data_ = pinocchio::Data(model_);

        // Find the end effector frame
        end_effector_id_ = model_.getFrameId(end_effector_name);

        if (end_effector_id_ >= model_.nframes) {
            std::cerr << "Error: End effector frame '" << end_effector_name
                      << "' not found in URDF!" << std::endl;
            throw std::runtime_error("End effector frame not found");
        }
        std::cout << "Successfully loaded URDF from: " << urdf_path
                  << std::endl;
        std::cout << "Model has " << model_.nframes << " frames" << std::endl;
        std::cout << "Model has " << model_.nq << " configuration variables"
                  << std::endl;
        std::cout << "Model has " << model_.nv << " velocity variables"
                  << std::endl;
        std::cout << "End effector frame: '" << end_effector_name
                  << "' (ID: " << end_effector_id_ << ")" << std::endl;
        // Initialize joint limits from the URDF model
        joint_limits_lower_ = model_.lowerPositionLimit.cast<double>();
        joint_limits_upper_ = model_.upperPositionLimit.cast<double>();
        joint_limits_initialized_ = true;
        std::cout << "Joint limits loaded:" << std::endl;
        for (int i = 0; i < joint_limits_lower_.size(); ++i) {
            std::cout << "  Joint " << i << ": [" << joint_limits_lower_(i)
                      << ", " << joint_limits_upper_(i) << "]" << std::endl;
        }
        // Compute and print the end effector pose in neutral configuration
        computeForwardKinematics(NEUTRAL_JOINT_CONFIG);
        std::cout << "End effector pose (neutral config): ["
                  << getEndEffectorFrame() << "]" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error loading URDF: " << e.what() << std::endl;
        throw;
    }
}

void IKModel::printAllFrames() const {
    std::cout << "\n=== All Frames in Robot Model ===" << std::endl;
    std::cout << std::setw(30) << std::left << "Frame Name" << std::setw(15)
              << std::left << "Type" << std::setw(10) << std::left << "ID"
              << std::setw(10) << std::left << "Parent ID" << std::endl;
    std::cout << std::string(65, '-') << std::endl;

    for (pinocchio::FrameIndex i = 0; i < model_.nframes; ++i) {
        const pinocchio::Frame &frame = model_.frames[i];
        std::string frame_type;

        switch (frame.type) {
        case pinocchio::FrameType::OP_FRAME:
            frame_type = "OP_FRAME";
            break;
        case pinocchio::FrameType::JOINT:
            frame_type = "JOINT";
            break;
        case pinocchio::FrameType::FIXED_JOINT:
            frame_type = "FIXED_JOINT";
            break;
        case pinocchio::FrameType::BODY:
            frame_type = "BODY";
            break;
        case pinocchio::FrameType::SENSOR:
            frame_type = "SENSOR";
            break;
        default:
            frame_type = "UNKNOWN";
            break;
        }

        std::cout << std::setw(30) << std::left << frame.name << std::setw(15)
                  << std::left << frame_type << std::setw(10) << std::left << i
                  << std::setw(10) << std::left << frame.parentJoint
                  << std::endl;
    }
    std::cout << std::endl;
}

void IKModel::findAndPrintFrame(const std::string &frame_name) {
    std::cout << "\n=== Searching for Frame: " << frame_name
              << " ===" << std::endl;

    // Search for the frame by name
    pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

    if (frame_id >= model_.nframes) {
        std::cout << "Frame '" << frame_name << "' not found!" << std::endl;
        return;
    }

    const pinocchio::Frame &frame = model_.frames[frame_id];

    // Determine frame type
    std::string frame_type;
    switch (frame.type) {
    case pinocchio::FrameType::OP_FRAME:
        frame_type = "OP_FRAME";
        break;
    case pinocchio::FrameType::JOINT:
        frame_type = "JOINT";
        break;
    case pinocchio::FrameType::FIXED_JOINT:
        frame_type = "FIXED_JOINT";
        break;
    case pinocchio::FrameType::BODY:
        frame_type = "BODY";
        break;
    case pinocchio::FrameType::SENSOR:
        frame_type = "SENSOR";
        break;
    default:
        frame_type = "UNKNOWN";
        break;
    }

    // Print frame information
    std::cout << "Frame Name: " << frame.name << std::endl;
    std::cout << "Frame Type: " << frame_type << std::endl;
    std::cout << "Frame ID: " << frame_id << std::endl;
    std::cout << "Parent Joint ID: " << frame.parentJoint << std::endl;

    // Compute forward kinematics using current joint configuration
    pinocchio::forwardKinematics(model_, data_, current_joint_config_);
    pinocchio::updateFramePlacements(model_, data_);

    // Get the frame pose in world coordinates
    const pinocchio::SE3 &frame_pose = data_.oMf[frame_id];

    // Create the XYZQuat representation
    XYZQuat xyzquat;
    xyzquat.head<3>() = frame_pose.translation();

    // Convert rotation matrix to quaternion (x, y, z, w)
    Eigen::Quaterniond quat(frame_pose.rotation());
    xyzquat.segment<4>(3) =
        Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w());
    // Print the frame pose in XYZQuat format
    std::cout << "Frame Pose (world coordinates, XYZQuat): ["
              << xyzquat.transpose() << "]" << std::endl;
}

pinocchio::SE3 IKModel::computeForwardKinematics(const JointConfig &q) {
    // Check if joint configuration has correct size
    if (model_.nq != 6) {
        throw std::runtime_error("Model configuration size (" +
                                 std::to_string(model_.nq) +
                                 ") does not match expected size (6)");
    }

    // Compute forward kinematics
    pinocchio::forwardKinematics(model_, data_, q);

    // Update frame placements
    pinocchio::updateFramePlacements(model_, data_);

    // Get the end effector transformation
    // const pinocchio::SE3 &end_effector_pose =
    return data_.oMf[end_effector_id_];
}

XYZQuat IKModel::homogeneousToXYZQuat(const pinocchio::SE3 &se3_transform) {
    XYZQuat xyzquat;

    // Extract translation (x, y, z)
    xyzquat.head<3>() = se3_transform.translation();

    // Extract rotation matrix and convert to quaternion
    Eigen::Quaterniond quat(se3_transform.rotation());

    // Store quaternion as [qx, qy, qz, qw]
    xyzquat.segment<4>(3) =
        Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w());

    return xyzquat;
}

pinocchio::SE3 IKModel::xyzQuatToHomogeneous(const XYZQuat &xyzquat) {
    // Extract translation
    Eigen::Vector3d translation = xyzquat.head<3>();

    // Convert quaternion to rotation matrix
    Eigen::Quaterniond quat(xyzquat(3), xyzquat(4), xyzquat(5),
                            xyzquat(6)); // x, y, z, w
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

    // Create SE3 transform
    return pinocchio::SE3(rotation_matrix, translation);
}

JointConfig IKModel::cost_grad(const JointConfig &q,
                               const pinocchio::SE3 &target_pose) {
    // 1) FK & error as before
    pinocchio::SE3 fk_pose = computeForwardKinematics(q);
    // 1) compute the error transform
    pinocchio::SE3 T_err = target_pose.inverse() * fk_pose;

    // 2) 6-vector error
    auto err6 = pinocchio::log6(T_err);
    Eigen::Matrix<double, 6, 1> e = err6.toVector();

    // 3) 6×n frame Jacobian at fk_pose
    Eigen::Matrix<double, 6, Eigen::Dynamic> J6(6, q.size());
    pinocchio::computeFrameJacobian(model_, data_, q, end_effector_id_,
                                    pinocchio::ReferenceFrame::LOCAL, J6);

    // 4) **inverse left‐Jacobian** Jₗ⁻¹ of the log‐map, built from T_err
    Eigen::Matrix<double, 6, 6> Jl_inv = pinocchio::Jlog6(T_err);

    // 5) total error‐Jacobian Je = Jₗ⁻¹ · J₆
    Eigen::Matrix<double, 6, Eigen::Dynamic> Je = Jl_inv * J6;

    // 6) gradient and Gauss–Newton Hessian
    return Je.transpose() * pose_weights_ * e;
    // Eigen::MatrixXd hess = Je.transpose() * pose_weights_ * Je;
}

Eigen::Matrix<double, 6, 6>
IKModel::cost_hess(const JointConfig &q, const pinocchio::SE3 &target_pose) {
    // 1) FK & error as before
    pinocchio::SE3 fk_pose = computeForwardKinematics(q);
    // 1) compute the error transform
    pinocchio::SE3 T_err = target_pose.inverse() * fk_pose;

    // 2) 6-vector error
    auto err6 = pinocchio::log6(T_err);
    Eigen::Matrix<double, 6, 1> e = err6.toVector();

    // 3) 6×n frame Jacobian at fk_pose
    Eigen::Matrix<double, 6, Eigen::Dynamic> J6(6, q.size());
    pinocchio::computeFrameJacobian(model_, data_, q, end_effector_id_,
                                    pinocchio::ReferenceFrame::LOCAL, J6);

    // 4) **inverse left‐Jacobian** Jₗ⁻¹ of the log‐map, built from T_err
    Eigen::Matrix<double, 6, 6> Jl_inv = pinocchio::Jlog6(T_err);

    // 5) total error‐Jacobian Je = Jₗ⁻¹ · J₆
    Eigen::Matrix<double, 6, Eigen::Dynamic> Je = Jl_inv * J6;

    // 6) gradient and Gauss–Newton Hessian
    return Je.transpose() * pose_weights_ * Je;
}

void IKModel::warmStartOptimization(const pinocchio::SE3 &target_pose, int its,
                                    double lambda_damping) {
    const double eps = 0.001; // 1mm tolerance
    const double gain = 1.0;
    const double alpha = 0.1;
    const double epsilon = 1e-4; // damping for pseudo-inverse

    std::cout << "Position-only warm start optimization started!" << std::endl;

    // Use current joint configuration as starting point
    JointConfig q = current_joint_config_;

    // Extract desired end effector position from target pose
    Eigen::Vector3d end_eff_desired = target_pose.translation();

    bool success = false;

    for (int i = 0; i < its; ++i) {
        // Compute forward kinematics
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        // Get the current end effector pose
        const pinocchio::SE3 &T_S_F = data_.oMf[end_effector_id_];

        // Compute position error
        Eigen::Vector3d err = gain * (end_eff_desired - T_S_F.translation());

        // Check convergence
        double err_norm = err.norm();
        if (err_norm < eps) {
            success = true;
            std::cout << "Position-only IK converged in " << i << " steps"
                      << std::endl;
            std::cout << "Found joint positions: [" << q.transpose() << "]"
                      << std::endl;
            std::cout << "End effector position: ["
                      << T_S_F.translation().transpose() << "]" << std::endl;
            std::cout << "Desired position: [" << end_eff_desired.transpose()
                      << "]" << std::endl;
            break;
        }

        if (i == its - 1) {
            std::cout << "Position-only IK: no good solution found"
                      << std::endl;
            std::cout << "Current joint positions: [" << q.transpose() << "]"
                      << std::endl;
            std::cout << "End effector position: ["
                      << T_S_F.translation().transpose() << "]" << std::endl;
            std::cout << "Desired position: [" << end_eff_desired.transpose()
                      << "]" << std::endl;
        }

        // Compute frame Jacobian
        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, q.size());
        pinocchio::computeFrameJacobian(model_, data_, q, end_effector_id_,
                                        pinocchio::ReferenceFrame::LOCAL, J);

        // Extract position Jacobian (first 3 rows) and transform to spatial
        // frame
        Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos =
            T_S_F.rotation() * J.topRows<3>();

        // Compute damped pseudo-inverse
        Eigen::Matrix<double, 3, 3> JJt = J_pos * J_pos.transpose();
        JJt.diagonal().array() += epsilon;
        Eigen::Matrix<double, Eigen::Dynamic, 3> pinv =
            J_pos.transpose() * JJt.inverse();

        // Compute joint update
        Eigen::VectorXd dq = pinv * err;

        // Update joint configuration
        q = q + alpha * dq;

        // Clamp to joint limits
        q = q.cwiseMax(joint_limits_lower_).cwiseMin(joint_limits_upper_);
    }

    // Update current joint configuration with the result
    current_joint_config_ = q;

    if (!success) {
        std::cout << "Position-only warm start optimization completed (max "
                     "iterations reached)"
                  << std::endl;
    }
}

// void IKModel::warmStartOptimization(const pinocchio::SE3 &target_pose, int
// its,
//                                     double lambda_damping) {
//     const double eps = 1e-4;
//     const double DT = 1e-1;
//     const double damp = lambda_damping;
//     std::cout << "Warm start optimization started!" << std::endl;
//     // Use current joint configuration as starting point
//     JointConfig q = current_joint_config_;

//     // Initialize Jacobian matrix
//     pinocchio::Data::Matrix6x J(6, model_.nv);
//     J.setZero();

//     bool success = false;
//     typedef Eigen::Matrix<double, 6, 1> Vector6d;
//     Vector6d err;
//     Eigen::VectorXd v(model_.nv);

//     for (int i = 0; i < its; ++i) {
//         // Forward kinematics
//         pinocchio::forwardKinematics(model_, data_, q);

//         // Compute error in end effector frame
//         const pinocchio::SE3 iMd =
//             data_.oMf[end_effector_id_].actInv(target_pose);
//         err = pinocchio::log6(iMd).toVector(); // in joint frame

//         if (err.norm() < eps) {
//             success = true;
//             break;
//         }

//         // Compute Jacobian in joint frame
//         pinocchio::computeFrameJacobian(model_, data_, q, end_effector_id_,
//                                         pinocchio::ReferenceFrame::LOCAL, J);

//         // Compute Jlog6 matrix
//         pinocchio::Data::Matrix6 Jlog;
//         pinocchio::Jlog6(iMd.inverse(), Jlog);
//         J = -Jlog * J;

//         // Solve for velocity update
//         pinocchio::Data::Matrix6 JJt;
//         JJt.noalias() = J * J.transpose();
//         JJt.diagonal().array() += damp;
//         v.noalias() = -J.transpose() * JJt.ldlt().solve(err);

//         // Integrate joint configuration
//         q = pinocchio::integrate(model_, q, v * DT);

//         // Clamp to joint limits
//         q = q.cwiseMax(joint_limits_lower_).cwiseMin(joint_limits_upper_);
//     }

//     // Update current joint configuration with the result
//     current_joint_config_ = q;

//     if (success) {
//         std::cout << "Warm start optimization converged!" << std::endl;
//     } else {
//         std::cout
//             << "Warm start optimization completed (max iterations reached)"
//             << std::endl;
//     }
// }
// void IKModel::warmStartOptimization(const pinocchio::SE3 &target_pose, int
// its,
//                                     double lambda_damping) {
//     for (int i = 0; i < its; ++i) {
//         auto J = computeGeometricJacobian(current_joint_config_);
//         auto err =
//             pinocchio::log6(data_.oMf[end_effector_id_].inverse() *
//             target_pose)
//                 .toVector();
//         auto H = J.transpose() * J +
//                  lambda_damping * Eigen::Matrix<double, 6, 6>::Identity();
//         auto delta = H.ldlt().solve(J.transpose() * err);
//         current_joint_config_ += delta;
//         current_joint_config_ =
//             current_joint_config_.cwiseMax(joint_limits_lower_)
//                 .cwiseMin(joint_limits_upper_);
//     }
//     std::cout << "Warm start optimization completed!" << std::endl;
//     // std::cout << "Current joint configuration: ["
//     //           << current_joint_config_.transpose() * (180 / M_PI) << "]
//     deg"
//     //           << std::endl;
// }