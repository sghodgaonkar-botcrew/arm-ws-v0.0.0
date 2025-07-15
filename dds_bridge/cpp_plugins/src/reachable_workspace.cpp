#include "ik_model.h"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/collision/collision.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <Eigen/Geometry>
#include <boost/random/sobol.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <random>
#include <sstream>
#include <vector>

const std::string robots_model_path =
    "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/";
const std::string urdf_filename =
    "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf";
const std::string srdf_filename =
    "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.srdf";
const std::string end_effector_name = "connection_frame";

const long unsigned int num_points = 500000;

using JointConfig = Eigen::Vector<double, 6>;

// Structure to hold workspace point data with position, rotation, and joint
// config
struct WorkspacePoint {
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    JointConfig joint_config;

    WorkspacePoint(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot,
                   const JointConfig &config)
        : position(pos), rotation(rot), joint_config(config) {}
};

// Helper function to process a single sample
bool processSample(const pinocchio::Model &model, pinocchio::Data &data,
                   const pinocchio::GeometryModel &geom_model,
                   pinocchio::GeometryData &geom_data,
                   const Eigen::VectorXd &joint_limits_lower,
                   const Eigen::VectorXd &joint_limits_upper,
                   const int frame_id, boost::random::sobol &seq,
                   std::vector<double> &u, JointConfig &q,
                   WorkspacePoint &result) {

    // Safety checks
    if (frame_id < 0 || frame_id >= model.frames.size()) {
        return false;
    }

    if (u.size() != model.nq) {
        return false;
    }

    // Generate Sobol sequence values for the entire point at once
    seq.generate(u.begin(), u.end());

    // Map to joint configuration with bounds checking
    for (int j = 0; j < model.nq; ++j) {
        if (j < joint_limits_lower.size() && j < joint_limits_upper.size()) {
            q[j] = joint_limits_lower[j] +
                   ((joint_limits_upper[j] - joint_limits_lower[j]) * u[j]);
        } else {
            return false; // Invalid joint limits
        }
    }

    try {
        // Compute forward kinematics
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data);

        // Check for collisions
        bool collision_detected =
            pinocchio::computeCollisions(model, data, geom_model, geom_data, q);

        // Only return collision-free configurations above ground
        if (!collision_detected) {
            Eigen::Vector3d position = data.oMf[frame_id].translation();
            // if (position[2] > 0.0) {
            // Extract rotation matrix and convert to quaternion
            Eigen::Matrix3d rotation_matrix = data.oMf[frame_id].rotation();
            Eigen::Quaterniond rotation(rotation_matrix);
            rotation.normalize(); // Ensure unit quaternion

            result = WorkspacePoint(position, rotation, q);
            return true;
            // }
        }
    } catch (const std::exception &e) {
        // Log error and return false
        return false;
    } catch (...) {
        // Catch any other exceptions
        return false;
    }

    return false;
}

// Helper function to write binary PLY file
void writeBinaryPLY(const std::string &filename,
                    const std::vector<WorkspacePoint> &workspace_points) {
    std::ofstream ply_file(filename, std::ios::binary);

    if (!ply_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename
                  << " for writing." << std::endl;
        return;
    }

    // Write PLY header
    ply_file << "ply" << std::endl;
    ply_file << "format binary_little_endian 1.0" << std::endl;
    ply_file << "element vertex " << workspace_points.size() << std::endl;
    ply_file << "property float x" << std::endl;
    ply_file << "property float y" << std::endl;
    ply_file << "property float z" << std::endl;
    ply_file << "property float joint1" << std::endl;
    ply_file << "property float joint2" << std::endl;
    ply_file << "property float joint3" << std::endl;
    ply_file << "property float joint4" << std::endl;
    ply_file << "property float joint5" << std::endl;
    ply_file << "property float joint6" << std::endl;
    ply_file << "end_header" << std::endl;

    // Write binary data
    for (const auto &point : workspace_points) {
        // Write position (3 floats)
        float x = static_cast<float>(point.position[0]);
        float y = static_cast<float>(point.position[1]);
        float z = static_cast<float>(point.position[2]);
        ply_file.write(reinterpret_cast<const char *>(&x), sizeof(float));
        ply_file.write(reinterpret_cast<const char *>(&y), sizeof(float));
        ply_file.write(reinterpret_cast<const char *>(&z), sizeof(float));

        // Write joint configuration (6 floats)
        for (int j = 0; j < point.joint_config.size(); ++j) {
            float joint_val = static_cast<float>(point.joint_config[j]);
            ply_file.write(reinterpret_cast<const char *>(&joint_val),
                           sizeof(float));
        }
    }

    ply_file.close();
    std::cout << "\nWorkspace points saved to: " << filename << std::endl;
    std::cout << "You can now load this file in CloudCompare to visualize the "
                 "workspace."
              << std::endl;
}

// Helper function to write k-d tree data to binary file
void writeKDTreeData(const std::string &filename,
                     const std::vector<WorkspacePoint> &workspace_points) {
    std::ofstream kdtree_file(filename, std::ios::binary);

    if (!kdtree_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename
                  << " for writing." << std::endl;
        return;
    }

    // Write header information
    size_t num_points = workspace_points.size();
    kdtree_file.write(reinterpret_cast<const char *>(&num_points),
                      sizeof(size_t));

    // Write data for each point
    for (const auto &point : workspace_points) {
        // Write position (3 doubles)
        double x = point.position[0];
        double y = point.position[1];
        double z = point.position[2];
        kdtree_file.write(reinterpret_cast<const char *>(&x), sizeof(double));
        kdtree_file.write(reinterpret_cast<const char *>(&y), sizeof(double));
        kdtree_file.write(reinterpret_cast<const char *>(&z), sizeof(double));

        // Write quaternion (4 doubles: w, x, y, z)
        double qw = point.rotation.w();
        double qx = point.rotation.x();
        double qy = point.rotation.y();
        double qz = point.rotation.z();
        kdtree_file.write(reinterpret_cast<const char *>(&qw), sizeof(double));
        kdtree_file.write(reinterpret_cast<const char *>(&qx), sizeof(double));
        kdtree_file.write(reinterpret_cast<const char *>(&qy), sizeof(double));
        kdtree_file.write(reinterpret_cast<const char *>(&qz), sizeof(double));

        // Write joint configuration (6 doubles)
        for (int j = 0; j < point.joint_config.size(); ++j) {
            double joint_val = point.joint_config[j];
            kdtree_file.write(reinterpret_cast<const char *>(&joint_val),
                              sizeof(double));
        }
    }

    kdtree_file.close();
    std::cout << "K-d tree data saved to: " << filename << std::endl;
    std::cout << "File contains " << workspace_points.size()
              << " points with position, rotation (quaternion), and joint "
                 "configuration."
              << std::endl;
}

// Helper function to get timestamped filename
std::string getTimestampedFilename(const std::string &base_dir,
                                   const std::string &prefix,
                                   const std::string &extension) {
    // Create output directory if it doesn't exist
    if (!std::filesystem::exists(base_dir)) {
        std::filesystem::create_directory(base_dir);
    }

    // Get current time for timestamp
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &now_c);
#else
    localtime_r(&now_c, &tm_buf);
#endif
    std::ostringstream oss;
    oss << base_dir << "/" << prefix << "_"
        << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << extension;
    return oss.str();
}

int main() {
    if (!std::filesystem::exists(urdf_filename)) {
        std::cerr << "ERROR: URDF file not found: " << urdf_filename
                  << std::endl;
        return -1;
    }
    if (!std::filesystem::exists(srdf_filename)) {
        std::cerr << "ERROR: SRDF file not found: " << srdf_filename
                  << std::endl;
        return -1;
    }

    using namespace pinocchio;

    // Load the robot model
    Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    std::cout << "Loaded model with " << model.nq << " joints" << std::endl;

    // Build the data associated to the model
    Data data(model);

    // set joint limits
    const auto joint_limits_lower = model.lowerPositionLimit.cast<double>();
    const auto joint_limits_upper = model.upperPositionLimit.cast<double>();

    const int frame_id = model.getFrameId(end_effector_name);

    // Load the geometries associated to model which are contained in the URDF
    // file
    GeometryModel geom_model;
    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION,
                               geom_model, robots_model_path);
    std::cout << "Loaded " << geom_model.ngeoms << " geometry objects"
              << std::endl;
    geom_model.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(model, geom_model, srdf_filename);
    GeometryData geom_data(geom_model);

    // Set a joint configuration (using neutral configuration from IKModel)
    JointConfig q = IKModel::NEUTRAL_JOINT_CONFIG;

    // Compute forward kinematics
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::updateGeometryPlacements(model, data, geom_model, geom_data);

    // Compute collisions
    bool collision_detected =
        pinocchio::computeCollisions(model, data, geom_model, geom_data, q);

    std::cout << "Forward kinematics computed for configuration" << std::endl;
    std::cout << "Collision detected: " << (collision_detected ? "YES" : "NO")
              << std::endl;

    // Print end effector pose
    std::cout << "End effector pose " << frame_id << ":" << std::endl;
    std::cout << data.oMf[frame_id] << std::endl;

    // Print information about collision pairs
    std::cout << "Number of collision pairs: "
              << geom_model.collisionPairs.size() << std::endl;

    // Check for collisions in each pair
    for (const auto &cp : geom_model.collisionPairs) {
        // Use the collision results from the initial computeCollisions() call
        bool pair_collision =
            geom_data.collisionResults[&cp - &geom_model.collisionPairs[0]]
                .isCollision();

        if (pair_collision) {
            // Get the names of the colliding geometry objects
            const std::string &name1 =
                geom_model.geometryObjects[cp.first].name;
            const std::string &name2 =
                geom_model.geometryObjects[cp.second].name;

            std::cout << "Collision pair: " << cp.first << " , " << cp.second
                      << " - COLLISION DETECTED" << std::endl;
            std::cout << "  Colliding objects: " << name1 << " <-> " << name2
                      << std::endl;
        }
    }

    std::cout << "Generating " << num_points
              << " random joint configurations..." << std::endl;

    // Progress reporting interval (1% of total)
    const long unsigned int progress_interval = std::max(1UL, num_points / 100);

    // Thread-local storage for workspace points
    std::vector<std::vector<WorkspacePoint>> thread_local_points;

    // Temporarily disable OpenMP to isolate segmentation fault
    int num_threads = 1;
    std::cout << "Using single-threaded processing for stability" << std::endl;

    try {
        thread_local_points.resize(num_threads);
        std::cout << "Thread-local storage allocated for " << num_threads
                  << " threads" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Failed to allocate thread-local storage: " << e.what()
                  << std::endl;
        return -1;
    }

    // Pre-allocate thread-local buffers with smaller chunks
    for (int i = 0; i < num_threads; ++i) {
        try {
            thread_local_points[i].reserve(std::min(
                10000UL,
                num_points / (num_threads * 20))); // Very conservative estimate
            std::cout << "Thread " << i << " buffer pre-allocated" << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "Failed to pre-allocate buffer for thread " << i
                      << ": " << e.what() << std::endl;
            return -1;
        }
    }

    std::cout << "Starting single-threaded processing..." << std::endl;

    // Single-threaded processing for stability
    auto &local_points = thread_local_points[0];

    // Thread-local Pinocchio data structures
    pinocchio::Data data_local(model);
    pinocchio::GeometryData geom_data_local(geom_model);

    // Sobol sequence and working variables
    boost::random::sobol seq(model.nq);
    std::vector<double> u(model.nq);
    JointConfig q_local = q;
    WorkspacePoint result(Eigen::Vector3d::Zero(),
                          Eigen::Quaterniond::Identity(), JointConfig::Zero());

    for (long unsigned int i = 0; i < num_points; ++i) {
        try {
            // Process single sample
            if (processSample(model, data_local, geom_model, geom_data_local,
                              joint_limits_lower, joint_limits_upper, frame_id,
                              seq, u, q_local, result)) {
                local_points.push_back(result);
            }

            // Progress reporting
            if (i % progress_interval == 0) {
                std::cout << "Progress: " << (i * 100 / num_points) << "% ("
                          << i << "/" << num_points << ")\r";
            }
        } catch (const std::exception &e) {
            std::cerr << "Error at iteration " << i << ": " << e.what()
                      << std::endl;
        } catch (...) {
            std::cerr << "Unknown error at iteration " << i << std::endl;
        }
    }

    std::cout << "\nSingle-threaded processing completed with "
              << local_points.size() << " points." << std::endl;

    // Merge all thread-local results
    std::vector<WorkspacePoint> all_workspace_points;
    size_t total_points = 0;
    for (const auto &thread_points : thread_local_points) {
        total_points += thread_points.size();
    }
    all_workspace_points.reserve(total_points);

    for (const auto &thread_points : thread_local_points) {
        all_workspace_points.insert(all_workspace_points.end(),
                                    thread_points.begin(), thread_points.end());
    }

    std::cout << "\nGenerated " << all_workspace_points.size()
              << " workspace points." << std::endl;

    // Example: Print first few entries
    std::cout << "\nFirst 5 workspace points:" << std::endl;
    int count = 0;
    for (const auto &point : all_workspace_points) {
        if (count >= 5)
            break;
        std::cout << "Position: " << point.position.transpose()
                  << " | Rotation (quat): [" << point.rotation.w() << ", "
                  << point.rotation.x() << ", " << point.rotation.y() << ", "
                  << point.rotation.z() << "]"
                  << " | Joint config: " << point.joint_config.transpose()
                  << std::endl;
        count++;
    }

    // Save workspace points to binary PLY file
    const std::string output_dir = "generated_workspaces";
    std::string ply_filename =
        getTimestampedFilename(output_dir, "workspace_points", ".ply");
    writeBinaryPLY(ply_filename, all_workspace_points);

    // Save k-d tree data to binary file
    std::string kdtree_filename =
        getTimestampedFilename(output_dir, "kdtree_data", ".bin");
    writeKDTreeData(kdtree_filename, all_workspace_points);

    return 0;
}