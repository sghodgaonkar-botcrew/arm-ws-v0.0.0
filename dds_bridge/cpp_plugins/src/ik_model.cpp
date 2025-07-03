#include "ik_model.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <iostream>
#include <iomanip>

// Define static member
const JointConfig IKModel::NEUTRAL_JOINT_CONFIG = JointConfig::Zero();

IKModel::IKModel(const std::string& urdf_path, const std::string& end_effector_name, const XYZQuat& pose_weights) 
    : urdf_path_(urdf_path), end_effector_name_(end_effector_name), current_joint_config_(NEUTRAL_JOINT_CONFIG), pose_weights_(pose_weights.asDiagonal()) {
    try {
        // Load the URDF model
        pinocchio::urdf::buildModel(urdf_path, model_);
        
        // Create the data object
        data_ = pinocchio::Data(model_);
        
        // Find the end effector frame
        end_effector_id_ = model_.getFrameId(end_effector_name);
        
        if (end_effector_id_ >= model_.nframes) {
            std::cerr << "Error: End effector frame '" << end_effector_name << "' not found in URDF!" << std::endl;
            throw std::runtime_error("End effector frame not found");
        }
        std::cout << "Successfully loaded URDF from: " << urdf_path << std::endl;
        std::cout << "Model has " << model_.nframes << " frames" << std::endl;
        std::cout << "Model has " << model_.nq << " configuration variables" << std::endl;
        std::cout << "Model has " << model_.nv << " velocity variables" << std::endl;
        std::cout << "End effector frame: '" << end_effector_name << "' (ID: " << end_effector_id_ << ")" << std::endl;
    // Compute and print the end effector pose in neutral configuration
    computeForwardKinematics(NEUTRAL_JOINT_CONFIG);
    std::cout << "End effector pose (neutral config): [" << end_effector_xyzquat_.transpose() << "]" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error loading URDF: " << e.what() << std::endl;
        throw;
    }
}

void IKModel::printAllFrames() const {
    std::cout << "\n=== All Frames in Robot Model ===" << std::endl;
    std::cout << std::setw(30) << std::left << "Frame Name" 
              << std::setw(15) << std::left << "Type" 
              << std::setw(10) << std::left << "ID" 
              << std::setw(10) << std::left << "Parent ID" << std::endl;
    std::cout << std::string(65, '-') << std::endl;
    
    for (pinocchio::FrameIndex i = 0; i < model_.nframes; ++i) {
        const pinocchio::Frame& frame = model_.frames[i];
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
        
        std::cout << std::setw(30) << std::left << frame.name
                  << std::setw(15) << std::left << frame_type
                  << std::setw(10) << std::left << i
                  << std::setw(10) << std::left << frame.parentJoint << std::endl;
    }
    std::cout << std::endl;
}

void IKModel::findAndPrintFrame(const std::string& frame_name) {
    std::cout << "\n=== Searching for Frame: " << frame_name << " ===" << std::endl;
    
    // Search for the frame by name
    pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
    
    if (frame_id >= model_.nframes) {
        std::cout << "Frame '" << frame_name << "' not found!" << std::endl;
        return;
    }
    
    const pinocchio::Frame& frame = model_.frames[frame_id];
    
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
    const pinocchio::SE3& frame_pose = data_.oMf[frame_id];
    
    // Create the XYZQuat representation
    XYZQuat xyzquat;
    xyzquat.head<3>() = frame_pose.translation();
    
    // Convert rotation matrix to quaternion (w, x, y, z)
    Eigen::Quaterniond quat(frame_pose.rotation());
    xyzquat.segment<4>(3) = Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w());
    // Print the frame pose in XYZQuat format
    std::cout << "Frame Pose (world coordinates, XYZQuat): [" 
              << xyzquat.transpose() << "]" << std::endl;
}

XYZQuat IKModel::computeForwardKinematics(const JointConfig& q) {
    // Check if joint configuration has correct size
    if (model_.nq != 6) {
        throw std::runtime_error("Model configuration size (" + std::to_string(model_.nq) + 
                                ") does not match expected size (6)");
    }
    
    // Compute forward kinematics
    pinocchio::forwardKinematics(model_, data_, q);
    
    // Update frame placements
    pinocchio::updateFramePlacements(model_, data_);
    
    // Get the end effector transformation
    const pinocchio::SE3& end_effector_pose = data_.oMf[end_effector_id_];
    
    // Create the XYZQuat representation directly
    XYZQuat xyzquat;
    xyzquat.head<3>() = end_effector_pose.translation();
    
    // Convert rotation matrix to quaternion (w, x, y, z)
    Eigen::Quaterniond quat(end_effector_pose.rotation());
    xyzquat.segment<4>(3) = Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w());
    
    // Update the stored XYZQuat representation and current joint config
    end_effector_xyzquat_ = xyzquat;
    current_joint_config_ = q;
    
    return xyzquat;
}

int IKModel::getNumJoints() const {
    return model_.nq;
}

XYZQuat IKModel::homogeneousToXYZQuat(const Eigen::Matrix4d& homogeneous_transform) {
    XYZQuat xyzquat;
    
    // Extract translation (x, y, z)
    xyzquat.head<3>() = homogeneous_transform.block<3,1>(0,3);
    
    // Extract rotation matrix and convert to quaternion
    Eigen::Matrix3d rotation_matrix = homogeneous_transform.block<3,3>(0,0);
    Eigen::Quaterniond quat(rotation_matrix);
    
    // Store quaternion as [qw, qx, qy, qz]
    xyzquat.segment<4>(3) = Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w());
    
    return xyzquat;
}

Eigen::Matrix4d IKModel::xyzQuatToHomogeneous(const XYZQuat& xyzquat) {
    Eigen::Matrix4d homogeneous_transform = Eigen::Matrix4d::Identity();
    
    // Set translation
    homogeneous_transform.block<3,1>(0,3) = xyzquat.head<3>();
    
    // Convert quaternion to rotation matrix
    Eigen::Quaterniond quat(xyzquat(3), xyzquat(4), xyzquat(5), xyzquat(6)); // w, x, y, z
    homogeneous_transform.block<3,3>(0,0) = quat.toRotationMatrix();
    
    return homogeneous_transform;
}

double IKModel::cost(const JointConfig& q, const XYZQuat& target_pose) {
    // 1. Forward kinematics for the given joint config
    XYZQuat fk_pose = computeForwardKinematics(q);

    // 2. Extract translation and quaternion for both poses
    Eigen::Vector3d tc = fk_pose.head<3>();
    Eigen::Vector3d td = target_pose.head<3>();
    Eigen::Quaterniond qc(fk_pose(3), fk_pose(4), fk_pose(5), fk_pose(6)); // (w, x, y, z)
    Eigen::Quaterniond qd(target_pose(3), target_pose(4), target_pose(5), target_pose(6));

    // 3. Compute translation error
    Eigen::Vector3d r_t = tc - td;

    // 4. Compute orientation error using quaternion log map
    // Compute relative quaternion: q_err = qc^{-1} * qd
    Eigen::Quaterniond qc_inv = qc.conjugate();
    Eigen::Quaterniond q_err = qc_inv * qd;
    q_err.normalize();

    // Clamp w to [-1, 1] for numerical stability
    double w = std::max(-1.0, std::min(1.0, q_err.w()));
    double theta = 2.0 * std::acos(w);

    Eigen::Vector3d r_o = Eigen::Vector3d::Zero();
    if (theta > 1e-8) {
        double sin_half_theta = std::sin(theta / 2.0);
        if (std::abs(sin_half_theta) > 1e-8) {
            r_o = (theta / sin_half_theta) * Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z());
        }
    }

    // 5. Weighted cost: 0.5 * (r_t^T * r_t + r_o^T * R_o * r_o)
    double cost = 0.5 * (r_t.squaredNorm() + r_o.transpose() * pose_weights_.bottomRightCorner<3,3>() * r_o);
    return cost;
}
