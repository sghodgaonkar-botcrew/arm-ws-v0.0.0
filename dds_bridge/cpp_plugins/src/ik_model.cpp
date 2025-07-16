#include "ik_model.h"
#include <Eigen/src/Core/MatrixBase.h>
#include <iomanip>
#include <iostream>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

// Define static member
const JointConfig IKModel::NEUTRAL_JOINT_CONFIG = JointConfig::Zero();

IKModel::IKModel(const std::string &urdf_path,
                 const std::string &end_effector_name,
                 const Eigen::Vector<double, 6> &pose_weights)
    : urdf_path_(urdf_path), end_effector_name_(end_effector_name),
      current_joint_config_(NEUTRAL_JOINT_CONFIG),
      pose_weights_(pose_weights.asDiagonal()) {
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

    // // Create the XYZQuat representation directly
    // XYZQuat xyzquat;
    // xyzquat.head<3>() = end_effector_pose.translation();

    // // Convert rotation matrix to quaternion (x, y, z, w)
    // Eigen::Quaterniond quat(end_effector_pose.rotation());
    // xyzquat.segment<4>(3) =
    //     Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w());

    // // Update the stored XYZQuat representation and current joint config
    // end_effector_xyzquat_ = xyzquat;
    // current_joint_config_ = q;

    // return xyzquat;
}

int IKModel::getNumJoints() const { return model_.nq; }

int IKModel::getNumVelocityVariables() const { return model_.nv; }

XYZQuat IKModel::homogeneousToXYZQuat(const pinocchio::SE3 &se3_transform) {
    XYZQuat xyzquat;

    // Extract translation (x, y, z)
    xyzquat.head<3>() = se3_transform.translation();

    // Extract rotation matrix and convert to quaternion
    Eigen::Matrix3d rotation_matrix = se3_transform.rotation();
    Eigen::Quaterniond quat(rotation_matrix);

    // Store quaternion as [qx, qy, qz, qw]
    xyzquat.segment<4>(3) =
        Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w());

    return xyzquat;
}

pinocchio::SE3 IKModel::xyzQuatToHomogeneous(const XYZQuat &xyzquat) {
    // Extract translation
    Eigen::Vector3d translation = xyzquat.head<3>();

    // Convert quaternion to rotation matrix
    // Eigen::Quaterniond quat(xyzquat(3), xyzquat(4), xyzquat(5),
    //                         xyzquat(6)); // x, y, z, w
    Eigen::Quaterniond quat(xyzquat(6), xyzquat(3), xyzquat(4),
                            xyzquat(5)); // w, x, y, z
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

    // Create SE3 transform
    return pinocchio::SE3(rotation_matrix, translation);
}

double IKModel::cost(const JointConfig &q, const pinocchio::SE3 &target_pose) {
    // 1. Forward kinematics for the given joint config
    const pinocchio::SE3 fk_pose = computeForwardKinematics(q);

    // 2. Compute the 6D pose error using SE3 logarithm: log(T_target^-1 *
    // T_current)
    Eigen::Matrix<double, 6, 1> error =
        (pinocchio::log6(target_pose.inverse() * fk_pose)).toVector();

    // 3. Return cost
    return 0.5 * error.transpose() * pose_weights_ * error;
}

JointConfig IKModel::cost_grad(const JointConfig &q,
                               const pinocchio::SE3 &target_pose) {
    // 1. Compute the pose error, in the same way that it was done in cost()
    const pinocchio::SE3 fk_pose = computeForwardKinematics(q);

    // 2. Compute the 6D pose error using SE3 logarithm: log(T_target^-1 *
    // T_current)
    Eigen::Matrix<double, 6, 1> error =
        (pinocchio::log6(target_pose.inverse() * fk_pose)).toVector();

    // 3. Jacobian
    Eigen::Matrix<double, 6, Eigen::Dynamic> J6(6, q.size());
    pinocchio::computeFrameJacobian(model_, data_, q, end_effector_id_,
                                    pinocchio::ReferenceFrame::LOCAL, J6);

    // 4. Return gradient
    return J6.transpose() * pose_weights_ * error;
}

Eigen::Matrix<double, 6, 6> IKModel::cost_hess(const JointConfig &q) {
    // 1. Compute Jacobian
    Eigen::Matrix<double, 6, Eigen::Dynamic> J6(6, q.size());
    pinocchio::computeFrameJacobian(model_, data_, q, end_effector_id_,
                                    pinocchio::ReferenceFrame::LOCAL, J6);

    // 2. Return the Gauss-Newton approximation: J^T * W * J
    return J6.transpose() * pose_weights_ * J6;
}

Eigen::Matrix<double, 6, 6>
IKModel::computeGeometricJacobian(const JointConfig &q) {
    // Compute forward kinematics first
    pinocchio::forwardKinematics(model_, data_, q);

    // Compute the geometric Jacobian for the end effector frame
    pinocchio::computeFrameJacobian(model_, data_, q, end_effector_id_,
                                    pinocchio::ReferenceFrame::LOCAL, data_.J);

    // Return the 6x6 Jacobian matrix
    return data_.J;
}

Eigen::Matrix<double, 7, 6>
IKModel::computeEndEffectorFullJacobian(const Eigen::VectorXd &q) {
    // Ensure the input vector has the correct size (nq degrees of freedom)
    if (q.size() != model_.nq)
        throw std::invalid_argument(
            "computeEndEffectorFullJacobian: expected q.size()=" +
            std::to_string(model_.nq) + ", got " + std::to_string(q.size()));

    // 1) Perform forward kinematics to compute joint placements and velocities
    pinocchio::forwardKinematics(model_, data_, q);
    //    Compute joint Jacobians ∂oMi/∂q for each joint
    pinocchio::computeJointJacobians(model_, data_, q);
    //    Update frame placements so data_.oMf is valid for all frames
    pinocchio::updateFramePlacements(model_, data_);

    // 2) Extract the standard 6×nv geometric Jacobian for the end-effector
    //    Rows 0–2: linear part (∂p/∂q), Rows 3–5: angular part (∂ω/∂q)
    Eigen::Matrix<double, 6, 6> J6;
    pinocchio::getFrameJacobian(model_, data_, end_effector_id_,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                J6);

    // Split J6 into translational (Jv) and rotational (Jw) components
    Eigen::Matrix<double, 3, 6> Jv = J6.topRows<3>();    // 3×6
    Eigen::Matrix<double, 3, 6> Jw = J6.bottomRows<3>(); // 3×6

    // 3) Build the 4×3 mapping B(q) that relates angular velocity ω to
    // quaternion rates q̇:
    //      q̇ = ½·B(q)·ω, where ω = Jw * q̇
    //    First, extract the current end-effector quaternion in (w, x, y, z)
    //    order
    const auto &M = data_.oMf[end_effector_id_];
    Eigen::Quaterniond qr(M.rotation()); // normalize if needed: qr.normalize();

    //    Populate B matrix (4×3) according to quaternion kinematics
    Eigen::Matrix<double, 4, 3> B;
    B << -qr.x(), -qr.y(), -qr.z(), // q̇x terms
        qr.w(), -qr.z(), qr.y(),    // q̇y terms
        qr.z(), qr.w(), -qr.x(),    // q̇z terms
        -qr.y(), qr.x(), qr.w();    // q̇w terms
    B *= 0.5;                       // apply ½ factor

    // 4) Compute the quaternion portion of the Jacobian (4×6)
    //    Each column maps q̇ → quaternion time derivative
    Eigen::Matrix<double, 4, 6> Jq = B * Jw;

    // 5) Stack the translational and quaternion Jacobians into a full 7×6
    // matrix
    Eigen::Matrix<double, 7, 6> J7;
    J7.topRows<3>() = Jv;    // [ ẋ; ẏ; ż ]
    J7.bottomRows<4>() = Jq; // [ q̇x; q̇y; q̇z; q̇w ]

    // Return the full Jacobian (7×6 for 6-DOF robot)
    return J7;
}
