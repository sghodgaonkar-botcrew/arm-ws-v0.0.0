#ifndef IK_MODEL_H
#define IK_MODEL_H

#include <Eigen/Dense>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <string>

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
     * @param pose_weights 6D vector for pose weights [x, y, z, ωx, ωy, ωz]
     */
    explicit IKModel(const std::string &urdf_path,
                     const std::string &end_effector_name,
                     const Eigen::Vector<double, 6> &pose_weights);

    /**
     * @brief Print all loaded frames
     */
    void printAllFrames() const;

    /**
     * @brief Find a frame by name and print its information
     * @param frame_name Name of the frame to find
     */
    void findAndPrintFrame(const std::string &frame_name);

    /**
     * @brief Compute forward kinematics for given joint configuration
     * @param q Joint configuration vector
     * @return 4x4 homogeneous transformation matrix of the end effector pose
     */
    pinocchio::SE3 computeForwardKinematics(const JointConfig &q);

    /**
     * @brief Compute forward kinematics for a specific frame using current
     * joint configuration
     * @param frame_id ID of the frame to compute pose for
     * @return XYZQuat representation of the frame pose [x, y, z, qx, qy, qz,
     * qw]
     */
    XYZQuat computeForwardKinematics(pinocchio::FrameIndex frame_id);

    /**
     * @brief Get the number of joints in the model
     * @return Number of joints
     */
    int getNumJoints() const;

    /**
     * @brief Get the number of velocity variables in the model
     * @return Number of velocity variables
     */
    int getNumVelocityVariables() const;

    /**
     * @brief Convert homogeneous transform to XYZQuat representation
     * @param homogeneous_transform 4x4 homogeneous transformation matrix
     * @return XYZQuat vector [x, y, z, qx, qy, qz, qw]
     */
    static XYZQuat
    homogeneousToXYZQuat(const pinocchio::SE3 &homogeneous_transform);

    /**
     * @brief Convert XYZQuat representation to homogeneous transform
     * @param xyzquat Vector [x, y, z, qx, qy, qz, qw]
     * @return 4x4 homogeneous transformation matrix
     */
    static pinocchio::SE3 xyzQuatToHomogeneous(const XYZQuat &xyzquat);

    /**
     * @brief Compute the cost between a joint configuration and a target pose
     * (XYZQuat)
     * @param q Joint configuration
     * @param target_pose Target pose in XYZQuat [x, y, z, qx, qy, qz, qw]
     * @return The cost (double)
     */
    double cost(const JointConfig &q, const pinocchio::SE3 &target_pose);

    /**
     * @brief Compute the gradient of the cost function with respect to joint
     * configuration
     * @param q Joint configuration
     * @param target_pose Target pose in XYZQuat [x, y, z, qx, qy, qz, qw]
     * @return The gradient vector (7×1)
     */
    JointConfig cost_grad(const JointConfig &q,
                          const pinocchio::SE3 &target_pose);

    /**
     * @brief Compute the Gauss-Newton approximation of the Hessian of the cost
     * function
     * @param q Joint configuration
     * @return 6x6 Hessian matrix approximation
     */
    Eigen::Matrix<double, 6, 6> cost_hess(const JointConfig &q);

    /**
     * @brief Get the current joint configuration
     */
    const JointConfig &getCurrentJointConfig() const {
        return current_joint_config_;
    }

    /**
     * @brief Get the current end effector pose in XYZQuat format
     */
    const pinocchio::SE3 &getEndEffectorFrame() const {
        return data_.oMf[end_effector_id_];
    }

    /**
     * @brief Compute the geometric Jacobian of the end effector
     * @param q Joint configuration vector
     * @return 6x6 geometric Jacobian matrix (linear and angular velocities)
     */
    Eigen::Matrix<double, 6, 6> computeGeometricJacobian(const JointConfig &q);

    /**
     * @brief Compute the full 7×nv Jacobian mapping joint velocities to
     * end-effector pose rates
     * @param q Joint configuration vector (can be dynamic size)
     * @return 7×nv Jacobian matrix (position + quaternion rates)
     */
    Eigen::Matrix<double, 7, 6>
    computeEndEffectorFullJacobian(const Eigen::VectorXd &q);

  private:
    pinocchio::Model model_;                ///< Robot model
    pinocchio::Data data_;                  ///< Robot data
    std::string urdf_path_;                 ///< URDF file path
    std::string end_effector_name_;         ///< End effector frame name
    pinocchio::FrameIndex end_effector_id_; ///< End effector frame ID
    // XYZQuat end_effector_xyzquat_; ///< End effector pose as [x, y, z, qx,
    // qy,
    //                                ///< qz, qw]
    JointConfig current_joint_config_; ///< Current joint configuration
    const Eigen::Matrix<double, 6, 6>
        pose_weights_; ///< Diagonal weight matrix for pose components
};

#endif // IK_MODEL_H