#include "ik_model.h"
#include <iostream>
#include <fstream>
#include <vector>

int main(int argc, char* argv[]) {
    // if (argc != 2) {
    //     std::cout << "Usage: " << argv[0] << " <path_to_urdf>" << std::endl;
    //     std::cout << "Example: " << argv[0] << " ../../urdf/ur10_v1.1.4/ur10.urdf" << std::endl;
    //     return 1;
    // }
    
    // std::string urdf_path = argv[1];
    // Try to find the URDF file in the workspace
    std::string urdf_path;
    std::vector<std::string> possible_paths = {
        "./urdf/ur10_v1.1.3/ur10.urdf",
        "../urdf/ur10_v1.1.3/ur10.urdf",
        "../../urdf/ur10_v1.1.3/ur10.urdf",
        "../../../urdf/ur10_v1.1.3/ur10.urdf"
    };
    
    bool file_found = false;
    for (const auto& path : possible_paths) {
        std::ifstream file_check(path);
        if (file_check.good()) {
            urdf_path = path;
            file_found = true;
            std::cout << "Found URDF file at: " << path << std::endl;
            break;
        }
    }
    
    if (!file_found) {
        std::cerr << "Error: Could not find URDF file. Tried the following paths:" << std::endl;
        for (const auto& path : possible_paths) {
            std::cerr << "  " << path << std::endl;
        }
        return 1;
    }
    try {
        // Create IKModel instance
        std::cout << "Creating IKModel with URDF: " << urdf_path << std::endl;
        
        // Define pose weights [x, y, z, qw, qx, qy, qz]
        XYZQuat pose_weights;
        pose_weights << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;  // Equal weights for all components
        
        IKModel ik_model(urdf_path, "connection_frame", pose_weights);  // Using connection_frame as end effector
        
        // Print all frames
        ik_model.printAllFrames();
        
        // Test finding specific frames
        std::cout << "Testing frame search functionality:" << std::endl;
        
        // Try to find some common frames
        ik_model.findAndPrintFrame("robot_base");
        ik_model.findAndPrintFrame("joint_5");
        ik_model.findAndPrintFrame("tool0");
        ik_model.findAndPrintFrame("nonexistent_frame");
        
        // Test forward kinematics
        std::cout << "\n=== Testing Forward Kinematics ===" << std::endl;
        
        // Test with zero joint configuration
        JointConfig q_zero = JointConfig::Zero();
        std::cout << "Joint configuration (all zeros): [" << q_zero.transpose() << "]" << std::endl;
        
        XYZQuat xyzquat_zero = ik_model.computeForwardKinematics(q_zero);
        std::cout << "End effector XYZQuat (zero config): [" << xyzquat_zero.transpose() << "]" << std::endl;
        
        // Test with neutral joint configuration
        std::cout << "\n--- Testing Neutral Joint Configuration ---" << std::endl;
        JointConfig q_neutral;
        for (int i = 0; i < 6; ++i) {
            q_neutral(i) = IKModel::NEUTRAL_JOINT_CONFIG[i];
        }
        std::cout << "Neutral joint configuration: [" << q_neutral.transpose() << "]" << std::endl;
        
        XYZQuat xyzquat_neutral = ik_model.computeForwardKinematics(q_neutral);
        std::cout << "End effector XYZQuat (neutral config): [" << xyzquat_neutral.transpose() << "]" << std::endl;
        
        // Test conversion functions
        std::cout << "\n=== Testing Conversion Functions ===" << std::endl;
        
        // Convert XYZQuat to homogeneous transform for comparison
        Eigen::Matrix4d T_neutral = IKModel::xyzQuatToHomogeneous(xyzquat_neutral);
        std::cout << "XYZQuat to Homogeneous conversion:" << std::endl;
        std::cout << T_neutral << std::endl;
        
        // Convert back to XYZQuat to verify
        XYZQuat xyzquat_converted = IKModel::homogeneousToXYZQuat(T_neutral);
        std::cout << "Homogeneous to XYZQuat conversion:" << std::endl;
        std::cout << "XYZQuat: [" << xyzquat_converted.transpose() << "]" << std::endl;
        
        // Verify the conversion is correct (should be identical)
        double error = (xyzquat_neutral - xyzquat_converted).norm();
        std::cout << "Conversion error (should be ~0): " << error << std::endl;
        
        // Test with a custom XYZQuat
        std::cout << "\n--- Testing Custom XYZQuat ---" << std::endl;
        XYZQuat custom_xyzquat;
        custom_xyzquat << 0.5, 0.3, 0.8, 1.0, 0.0, 0.0, 0.0; // [x, y, z, qw, qx, qy, qz]
        std::cout << "Custom XYZQuat: [" << custom_xyzquat.transpose() << "]" << std::endl;
        
        Eigen::Matrix4d T_custom = IKModel::xyzQuatToHomogeneous(custom_xyzquat);
        std::cout << "Custom homogeneous transform:" << std::endl;
        std::cout << T_custom << std::endl;
        
        // Convert back to verify
        XYZQuat xyzquat_back = IKModel::homogeneousToXYZQuat(T_custom);
        std::cout << "Converted back to XYZQuat: [" << xyzquat_back.transpose() << "]" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 