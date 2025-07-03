#ifndef IK_MODEL_H
#define IK_MODEL_H

#include <string>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>

// Type aliases for fixed-size vectors
using JointConfig = Eigen::Vector<double, 6>;
using XYZQuat = Eigen::Vector<double, 7>;

class IKModel {
public:
    // Neutral joint configuration for UR10 robot (6 joints)
    static const JointConfig NEUTRAL_JOINT_CONFIG;
    
    /**
     * @brief Constructor to load a URDF file
     * @param urdf_path Path to the URDF file describing the robot
     * @param end_effector_name Name of the end effector frame in the URDF
     * @param pose_weights 7D vector for pose weights [x, y, z, qw, qx, qy, qz]
     */
    explicit IKModel(const std::string& urdf_path, const std::string& end_effector_name, const XYZQuat& pose_weights);

    /**
     * @brief Print all loaded frames
     */
    void printAllFrames() const;

    /**
     * @brief Find a frame by name and print its information
     * @param frame_name Name of the frame to find
     */
    void findAndPrintFrame(const std::string& frame_name);

    /**
     * @brief Compute forward kinematics for given joint configuration
     * @param q Joint configuration vector
     * @return XYZQuat representation of the end effector pose [x, y, z, qw, qx, qy, qz]
     */
    XYZQuat computeForwardKinematics(const JointConfig& q);

    /**
     * @brief Compute forward kinematics for a specific frame using current joint configuration
     * @param frame_id ID of the frame to compute pose for
     * @return XYZQuat representation of the frame pose [x, y, z, qw, qx, qy, qz]
     */
    XYZQuat computeForwardKinematics(pinocchio::FrameIndex frame_id);

    /**
     * @brief Get the number of joints in the model
     * @return Number of joints
     */
    int getNumJoints() const;

    /**
     * @brief Convert homogeneous transform to XYZQuat representation
     * @param homogeneous_transform 4x4 homogeneous transformation matrix
     * @return XYZQuat vector [x, y, z, qw, qx, qy, qz]
     */
    static XYZQuat homogeneousToXYZQuat(const Eigen::Matrix4d& homogeneous_transform);

    /**
     * @brief Convert XYZQuat representation to homogeneous transform
     * @param xyzquat Vector [x, y, z, qw, qx, qy, qz]
     * @return 4x4 homogeneous transformation matrix
     */
    static Eigen::Matrix4d xyzQuatToHomogeneous(const XYZQuat& xyzquat);

    /**
     * @brief Compute the cost between a joint configuration and a target pose (XYZQuat)
     * @param q Joint configuration
     * @param target_pose Target pose in XYZQuat [x, y, z, qw, qx, qy, qz]
     * @return The cost (double)
     */
    double cost(const JointConfig& q, const XYZQuat& target_pose);

private:
    
    pinocchio::Model model_;    ///< Robot model
    pinocchio::Data data_;      ///< Robot data
    std::string urdf_path_;     ///< URDF file path
    std::string end_effector_name_; ///< End effector frame name
    pinocchio::FrameIndex end_effector_id_; ///< End effector frame ID
    XYZQuat end_effector_xyzquat_; ///< End effector pose as [x, y, z, qw, qx, qy, qz]
    JointConfig current_joint_config_; ///< Current joint configuration
    const Eigen::Matrix<double, 7, 7> pose_weights_; ///< Diagonal weight matrix for pose components
};

#endif // IK_MODEL_H 