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
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <unordered_set>
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

// Simple function to check if two joint configurations are approximately equal
bool isJointConfigEqual(const JointConfig &lhs, const JointConfig &rhs,
                        double tolerance = 1e-3) {
    for (int i = 0; i < lhs.size(); ++i) {
        if (std::abs(lhs[i] - rhs[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

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
                   const int frame_id, const std::vector<double> &u,
                   JointConfig &q, WorkspacePoint &result,
                   const std::vector<JointConfig> &existing_configs,
                   const double ground_threshold = 0.05) {

    // Safety checks
    if (frame_id < 0 || frame_id >= model.frames.size()) {
        return false;
    }

    if (u.size() != model.nq) {
        return false;
    }

    // Map Sobol sequence values to joint configuration with bounds checking

    // Map Sobol sequence [0,1] to joint limits with proper bounds checking
    for (int j = 0; j < model.nq; ++j) {
        // Ensure u[j] is in [0,1] range (Sobol should generate this, but let's
        // be safe)
        double clamped_u = std::max(0.0, std::min(1.0, u[j]));

        // Linear interpolation from joint limits
        q[j] = joint_limits_lower[j] +
               (joint_limits_upper[j] - joint_limits_lower[j]) * clamped_u;

        // Final bounds check
        if (q[j] < joint_limits_lower[j] || q[j] > joint_limits_upper[j]) {
            std::cerr << "ERROR: Joint " << j << " out of bounds: " << q[j]
                      << " not in [" << joint_limits_lower[j] << ", "
                      << joint_limits_upper[j] << "]" << std::endl;
            return false;
        }
    }

    // Check if this joint configuration already exists in the workspace
    for (const auto &existing_config : existing_configs) {
        if (isJointConfigEqual(q, existing_config)) {
            return false; // Duplicate configuration, discard it
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

            // Check that frames from link2 onwards are above ground
            bool frames_above_ground = true;

            // Find link2 frame ID
            const int link2_frame_id = model.getFrameId("link_2");

            // If link2 is found, check all frames from link2 onwards
            if (link2_frame_id >= 0) {
                for (pinocchio::FrameIndex i = link2_frame_id;
                     i < model.nframes; ++i) {
                    if (data.oMf[i].translation()[2] <= ground_threshold) {
                        frames_above_ground = false;
                        // std::cout << "Frame '" << model.frames[i].name
                        //           << "' touched the ground threshold (z = "
                        //           << data.oMf[i].translation()[2]
                        //           << ") at joint config (deg): ["
                        //           << (q * (180.0 / M_PI)).transpose() << "]"
                        //           << std::endl;
                        break;
                    }
                }
            } else {
                // If link2 not found, fall back to checking end effector only
                if (data.oMf[frame_id].translation()[2] <= ground_threshold) {
                    frames_above_ground = false;
                }
            }

            if (!frames_above_ground) {
                return false;
            }

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

// Helper function to build and save sparse 3D voxel grid
void buildAndSaveVoxelGrid(const std::vector<WorkspacePoint> &workspace_points,
                           const std::string &output_dir) {
    if (workspace_points.empty()) {
        std::cerr << "No workspace points to build voxel grid from."
                  << std::endl;
        return;
    }

    std::cout << "Building sparse 3D voxel grid from "
              << workspace_points.size() << " workspace points..." << std::endl;

    // A. Quantize point-cloud into a 3D grid
    // 1. Compute axis-aligned bounding box
    Eigen::Vector3d minB(+1e9, +1e9, +1e9), maxB(-1e9, -1e9, -1e9);
    for (const auto &wp : workspace_points) {
        minB = minB.cwiseMin(wp.position);
        maxB = maxB.cwiseMax(wp.position);
    }

    std::cout << "Bounding box: min=[" << minB.transpose() << "], max=["
              << maxB.transpose() << "]" << std::endl;

    // 2. Choose a voxel size (e.g. 1 cm)
    double r = 0.01; // 1cm voxel size

    // 3. Compute grid dimensions
    Eigen::Vector3i dims = ((maxB - minB) / r).cast<int>().array() + 1;

    std::cout << "Grid dimensions: [" << dims.transpose() << "]" << std::endl;

    // B. Bin each sample into its grid cell and collect sparse data
    auto flatIndex = [&](const Eigen::Vector3i &idx) {
        return static_cast<size_t>(idx.x()) * dims.y() * dims.z() +
               static_cast<size_t>(idx.y()) * dims.z() +
               static_cast<size_t>(idx.z());
    };

    // Use a map to store only non-empty cells
    std::map<size_t, std::vector<size_t>> sparseGrid;

    for (size_t i = 0; i < workspace_points.size(); ++i) {
        Eigen::Vector3i idx = ((workspace_points[i].position - minB) / r)
                                  .cast<int>()
                                  .cwiseMax(Eigen::Vector3i::Zero())
                                  .cwiseMin(dims - Eigen::Vector3i::Ones());
        size_t cellIndex = flatIndex(idx);
        sparseGrid[cellIndex].push_back(i);
    }

    // Count non-empty cells
    size_t nonEmptyCells = sparseGrid.size();
    std::cout << "Non-empty cells: " << nonEmptyCells << std::endl;

    // C. Create sparse nearest neighbor data with limited BFS
    std::map<size_t, int> sparseNearestIdx;
    std::queue<std::pair<size_t, int>> q; // (cell_index, distance)
    const int maxDistance =
        3; // Only fill cells within 3 voxels of workspace points

    // Seed the queue with every non-empty cell
    for (const auto &cell : sparseGrid) {
        sparseNearestIdx[cell.first] = static_cast<int>(
            cell.second[0]);     // Pick first sample as representative
        q.push({cell.first, 0}); // Distance 0 for actual workspace cells
    }

    // BFS flood-fill with distance limit
    // 6-connected neighbors
    const int offs[6][3] = {{+1, 0, 0}, {-1, 0, 0}, {0, +1, 0},
                            {0, -1, 0}, {0, 0, +1}, {0, 0, -1}};

    while (!q.empty()) {
        auto [c, distance] = q.front();
        q.pop();

        // Stop if we've reached the maximum distance
        if (distance >= maxDistance) {
            continue;
        }

        // Decode c → (x,y,z)
        int x = static_cast<int>(c / (dims.y() * dims.z()));
        int y = static_cast<int>((c / dims.z()) % dims.y());
        int z = static_cast<int>(c % dims.z());

        for (const auto &o : offs) {
            int nx = x + o[0], ny = y + o[1], nz = z + o[2];
            if (nx < 0 || ny < 0 || nz < 0 || nx >= dims.x() ||
                ny >= dims.y() || nz >= dims.z()) {
                continue;
            }

            size_t nc = static_cast<size_t>(nx) * dims.y() * dims.z() +
                        static_cast<size_t>(ny) * dims.z() +
                        static_cast<size_t>(nz);

            if (sparseNearestIdx.find(nc) == sparseNearestIdx.end()) {
                // Inherit the same representative point
                sparseNearestIdx[nc] = sparseNearestIdx[c];
                q.push({nc, distance + 1});
            }
        }
    }

    // Count filled cells
    size_t filledCells = sparseNearestIdx.size();
    std::cout << "Filled cells after limited BFS: " << filledCells << std::endl;

    // D. Serialize sparse data for runtime
    std::string grid_filename =
        getTimestampedFilename(output_dir, "workspace_grid_sparse", ".bin");
    std::ofstream out(grid_filename, std::ios::binary);

    if (!out.is_open()) {
        std::cerr << "Error: Could not open file " << grid_filename
                  << " for writing." << std::endl;
        return;
    }

    // 1) Header: minB(3×double), r(double), dims(3×int), maxDistance(int),
    // numCells(size_t)
    out.write(reinterpret_cast<const char *>(minB.data()), sizeof(double) * 3);
    out.write(reinterpret_cast<const char *>(&r), sizeof(double));
    out.write(reinterpret_cast<const char *>(dims.data()), sizeof(int) * 3);
    out.write(reinterpret_cast<const char *>(&maxDistance), sizeof(int));

    size_t numCells = sparseNearestIdx.size();
    out.write(reinterpret_cast<const char *>(&numCells), sizeof(size_t));

    // 2) Sparse data: (cell_index, nearest_point_index) pairs
    for (const auto &cell : sparseNearestIdx) {
        out.write(reinterpret_cast<const char *>(&cell.first), sizeof(size_t));
        out.write(reinterpret_cast<const char *>(&cell.second), sizeof(int));
    }

    out.close();

    std::cout << "Sparse voxel grid saved to: " << grid_filename << std::endl;
    std::cout << "Grid parameters: voxel_size=" << r << "m, "
              << "dimensions=[" << dims.transpose() << "], "
              << "max_distance=" << maxDistance << " voxels, "
              << "total_cells=" << numCells << std::endl;
    std::cout << "Memory usage: " << (numCells * (sizeof(size_t) + sizeof(int)))
              << " bytes (vs " << (dims.x() * dims.y() * dims.z() * sizeof(int))
              << " bytes for dense grid)" << std::endl;
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

    std::cout << "Model has " << model.nq << " joints" << std::endl;
    std::cout << "Joint limits lower: " << joint_limits_lower.transpose()
              << std::endl;
    std::cout << "Joint limits upper: " << joint_limits_upper.transpose()
              << std::endl;
    std::cout << "JointConfig size: " << JointConfig::SizeAtCompileTime
              << std::endl;

    // Verify that model has exactly 6 joints (as expected for UR10)
    if (model.nq != 6) {
        std::cerr << "ERROR: Expected 6 joints for UR10, but model has "
                  << model.nq << " joints!" << std::endl;
        return -1;
    }

    // Verify joint limits are reasonable
    for (int i = 0; i < model.nq; ++i) {
        if (joint_limits_lower[i] >= joint_limits_upper[i]) {
            std::cerr << "ERROR: Invalid joint limits for joint " << i
                      << ": lower=" << joint_limits_lower[i]
                      << ", upper=" << joint_limits_upper[i] << std::endl;
            return -1;
        }
        if (std::abs(joint_limits_lower[i]) > 1000 ||
            std::abs(joint_limits_upper[i]) > 1000) {
            std::cerr << "WARNING: Suspicious joint limits for joint " << i
                      << ": lower=" << joint_limits_lower[i]
                      << ", upper=" << joint_limits_upper[i] << std::endl;
        }
    }

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

    // Vector to track existing joint configurations to avoid duplicates
    std::vector<JointConfig> existing_configs;
    long unsigned int duplicate_count = 0;

    // Random number generator and working variables
    std::mt19937 gen(42); // Fixed seed for reproducible results
    std::uniform_real_distribution<double> dis(0.0, 1.0);
    std::vector<double> u(model.nq);
    JointConfig q_local = q;
    WorkspacePoint result(Eigen::Vector3d::Zero(),
                          Eigen::Quaterniond::Identity(), JointConfig::Zero());

    // Debug: Print joint limits for reference
    std::cout << "Joint limits for mapping:" << std::endl;
    std::cout << "Lower: [";
    for (int j = 0; j < model.nq; ++j) {
        std::cout << joint_limits_lower[j];
        if (j < model.nq - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "Upper: [";
    for (int j = 0; j < model.nq; ++j) {
        std::cout << joint_limits_upper[j];
        if (j < model.nq - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    for (long unsigned int i = 0; i < num_points; ++i) {
        try {
            // Generate random values for this iteration
            for (int j = 0; j < model.nq; ++j) {
                u[j] = dis(gen);
            }

            // Debug: Print first few random values to see what's happening
            if (i < 5) {
                std::cout << "Iteration " << i << " Random values: [";
                for (int j = 0; j < std::min(6, (int)u.size()); ++j) {
                    std::cout << u[j];
                    if (j < 5)
                        std::cout << ", ";
                }
                std::cout << "]" << std::endl;

                // Also show the resulting joint configuration
                std::cout << "Iteration " << i << " Joint config: [";
                for (int j = 0; j < 6; ++j) {
                    double joint_val =
                        joint_limits_lower[j] +
                        (joint_limits_upper[j] - joint_limits_lower[j]) * u[j];
                    std::cout << joint_val;
                    if (j < 5)
                        std::cout << ", ";
                }
                std::cout << "]" << std::endl;
            }

            // Process single sample
            if (processSample(model, data_local, geom_model, geom_data_local,
                              joint_limits_lower, joint_limits_upper, frame_id,
                              u, q_local, result, existing_configs)) {
                local_points.push_back(result);
                existing_configs.push_back(
                    q_local); // Add to existing configurations
            } else {
                // Check if it was rejected due to being a duplicate
                bool is_duplicate = false;
                for (const auto &existing_config : existing_configs) {
                    if (isJointConfigEqual(q_local, existing_config)) {
                        is_duplicate = true;
                        duplicate_count++;
                        break;
                    }
                }
            }

            // Progress reporting
            if (i % progress_interval == 0) {
                std::cout << "Progress: " << (i * 100 / num_points) << "% ("
                          << i << "/" << num_points << ")\n";
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
    std::cout << "Duplicate configurations detected and skipped: "
              << duplicate_count << std::endl;
    std::cout << "Total unique joint configurations: "
              << existing_configs.size() << std::endl;

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

    // Build and save 3D voxel grid
    buildAndSaveVoxelGrid(all_workspace_points, output_dir);

    return 0;
}