/**
 * @file reachable_workspace_generator.cpp
 * @brief Implementation of Reachable Workspace Generator for Robot IK Solver
 *
 * This file implements the ReachableWorkspaceGenerator class, providing
 * comprehensive workspace sampling and generation capabilities for robot
 * inverse kinematics solving. The implementation includes efficient random
 * sampling, collision detection, duplicate handling, and multiple output
 * format generation.
 *
 * **Key Implementation Features:**
 * - Efficient hash-based duplicate detection
 * - Batch processing for improved performance
 * - Cross-platform timestamp generation
 * - Multiple output formats (PLY, binary, voxel grid)
 * - Comprehensive error handling and progress reporting
 *
 * **Performance Optimizations:**
 * - Pre-allocated vectors to avoid reallocation
 * - Hash set for O(1) duplicate detection
 * - Batch processing for better cache locality
 * - Binary I/O for efficient file operations
 *
 * @author Shantanu Ghodgaonkar
 * @date 2025-07-21
 * @version 1.0
 */

#include "reachable_workspace_generator.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>

/**
 * @brief Hash function implementation for JointConfig
 *
 * **Hash Algorithm:**
 * - Iterates through each joint angle in the configuration
 * - Combines hash values using XOR and bit shifting operations
 * - Uses golden ratio constant (0x9e3779b9) for better distribution
 * - Provides fast computation suitable for real-time applications
 *
 * **Hash Strategy:**
 * - Simple but effective combination of individual joint hashes
 * - Uses bit manipulation for good distribution properties
 * - Balances speed and collision resistance
 *
 * **Performance:**
 * - O(n) time complexity where n is number of joints
 * - Constant space complexity
 * - Optimized for frequent lookups in hash sets
 */
std::size_t JointConfigHash::operator()(const JointConfig &config) const {
    std::size_t hash = 0;
    for (int i = 0; i < config.size(); ++i) {
        // Simple hash combination using XOR and bit shifting
        // Golden ratio constant (0x9e3779b9) improves distribution
        hash ^= std::hash<double>{}(config[i]) + 0x9e3779b9 + (hash << 6) +
                (hash >> 2);
    }
    return hash;
}

/**
 * @brief Equality function implementation for JointConfig
 *
 * **Comparison Strategy:**
 * - Uses tolerance-based comparison (1e-3 radians)
 * - Handles floating-point precision errors gracefully
 * - Returns false on first difference exceeding tolerance
 * - Ensures robust duplicate detection in hash sets
 *
 * **Tolerance Selection:**
 * - 1e-3 radians ≈ 0.057 degrees
 * - Balances precision and numerical stability
 * - Prevents false negatives due to floating-point errors
 * - Suitable for most robotic applications
 *
 * **Performance:**
 * - O(n) time complexity where n is number of joints
 * - Early exit on first difference
 * - Optimized for frequent comparisons
 */
bool JointConfigEqual::operator()(const JointConfig &lhs,
                                  const JointConfig &rhs) const {
    const double tolerance = 1e-3;
    for (int i = 0; i < lhs.size(); ++i) {
        if (std::abs(lhs[i] - rhs[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Constructor implementation
 *
 * **Initialization Flow:**
 * 1. Store robot model reference for kinematics and collision checking
 * 2. Initialize random number generator with fixed seed (42) for
 * reproducibility
 * 3. Set all generation parameters from constructor arguments
 * 4. Print initialization summary with key parameters
 *
 * **Parameter Validation:**
 * - All parameters are stored as member variables
 * - No validation performed (assumes valid input)
 * - Default values provide reasonable starting points
 *
 * **Reproducibility:**
 * - Fixed seed (42) ensures reproducible results
 * - Important for debugging and testing
 * - Can be modified for different random sequences
 */
ReachableWorkspaceGenerator::ReachableWorkspaceGenerator(
    RobotModel &robot_model, long unsigned int num_points, int batch_size,
    double ground_threshold, double joint_tolerance,
    int max_attempts_per_sample, int max_distance)
    : robot_model_(robot_model), random_generator_(42), num_points_(num_points),
      batch_size_(batch_size), ground_threshold_(ground_threshold),
      joint_tolerance_(joint_tolerance),
      max_attempts_per_sample_(max_attempts_per_sample),
      max_distance_(max_distance) {
    std::cout << "Initialized workspace generator with external RobotModel"
              << std::endl;
    std::cout << "Workspace parameters: " << num_points_ << " points, "
              << batch_size_ << " batch size, " << ground_threshold_
              << " ground threshold, " << max_distance_ << " max distance"
              << std::endl;
}

/**
 * @brief Destructor implementation
 *
 * **Cleanup:**
 * - No manual cleanup required
 * - RobotModel handles its own resource management
 * - Member variables are automatically cleaned up
 * - Virtual destructor ensures proper cleanup in derived classes
 */
ReachableWorkspaceGenerator::~ReachableWorkspaceGenerator() {
    // No manual cleanup needed - RobotModel handles its own resources
}

/**
 * @brief Process a single joint configuration and compute workspace point
 *
 * **Processing Flow:**
 * 1. **Validation**: Check if configuration is valid using robot model
 * 2. **Forward Kinematics**: Compute end-effector pose
 * 3. **Data Extraction**: Extract position and rotation from SE3 transform
 * 4. **Quaternion Normalization**: Ensure unit quaternion
 * 5. **Result Creation**: Create and return WorkspacePoint
 *
 * **Error Handling:**
 * - Returns false for invalid configurations (collision, ground contact)
 * - Catches and reports exceptions during processing
 * - Provides detailed error messages for debugging
 * - Exception-safe implementation
 *
 * **Performance:**
 * - Fast processing for valid configurations
 * - Early exit for invalid configurations
 * - Minimal memory allocation
 * - Exception-safe design
 */
bool ReachableWorkspaceGenerator::processSampleFast(const JointConfig &q,
                                                    WorkspacePoint &result) {
    try {
        // Use RobotModel to check if configuration is valid
        if (!robot_model_.isValidConfiguration(q, ground_threshold_)) {
            return false;
        }

        // Compute forward kinematics using RobotModel
        pinocchio::SE3 end_effector_pose =
            robot_model_.computeForwardKinematics(q);

        // Extract position and rotation from SE3 transform
        Eigen::Vector3d position = end_effector_pose.translation();
        Eigen::Matrix3d rotation_matrix = end_effector_pose.rotation();
        Eigen::Quaterniond rotation(rotation_matrix);
        rotation.normalize(); // Ensure unit quaternion for numerical stability

        // Create workspace point with computed data
        result = WorkspacePoint(position, rotation, q);
        return true;
    } catch (const std::exception &e) {
        std::cerr << "Error processing sample: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Process a batch of random joint configurations
 *
 * **Batch Processing Algorithm:**
 * 1. **Random Generation**: Generate random joint configurations within limits
 * 2. **Duplicate Check**: Use hash set for O(1) duplicate detection
 * 3. **Retry Logic**: Retry with new random values for duplicates
 * 4. **Validation**: Process valid configurations using processSampleFast()
 * 5. **Collection**: Collect successful workspace points
 *
 * **Performance Optimizations:**
 * - Pre-allocates result vector (conservative estimate: batch_size/10)
 * - Uses hash set for fast duplicate detection
 * - Limits retry attempts to prevent infinite loops
 * - Batch processing improves cache locality
 *
 * **Duplicate Handling:**
 * - Marks duplicate configurations as seen to avoid infinite loops
 * - Retries with new random values up to max_attempts_per_sample
 * - Ensures progress even with high duplicate rates
 * - Uses tolerance-based comparison for numerical stability
 *
 * **Random Generation:**
 * - Uses uniform distribution over [0, 1]
 * - Maps to joint limits using linear interpolation
 * - Clamps values to ensure within bounds
 * - Provides good coverage of joint space
 */
std::vector<WorkspacePoint> ReachableWorkspaceGenerator::processBatch(
    std::mt19937 &gen,
    std::unordered_set<JointConfig, JointConfigHash, JointConfigEqual>
        &existing_configs,
    const int batch_size) {

    std::vector<WorkspacePoint> batch_results;
    batch_results.reserve(batch_size /
                          10); // Conservative estimate for valid configurations

    std::uniform_real_distribution<double> dis(0.0, 1.0);
    JointConfig q_local;
    WorkspacePoint result(Eigen::Vector3d::Zero(),
                          Eigen::Quaterniond::Identity(), JointConfig::Zero());

    int attempts = 0;
    const int max_attempts_per_sample = max_attempts_per_sample_;

    // Process each configuration in the batch
    for (int i = 0; i < batch_size; ++i) {
        attempts = 0;
        bool valid_config_found = false;

        // Keep trying until we find a unique configuration or hit max attempts
        while (attempts < max_attempts_per_sample && !valid_config_found) {
            // Generate random joint configuration within joint limits
            for (int j = 0; j < q_local.size(); ++j) {
                double u = dis(gen);
                double clamped_u = std::max(0.0, std::min(1.0, u));
                q_local[j] = robot_model_.getJointLimitsLower()[j] +
                             (robot_model_.getJointLimitsUpper()[j] -
                              robot_model_.getJointLimitsLower()[j]) *
                                 clamped_u;
            }

            // Check if configuration already exists using hash set
            if (existing_configs.find(q_local) != existing_configs.end()) {
                attempts++;
                continue; // Try again with new random values
            }

            // Configuration is unique, process it
            if (processSampleFast(q_local, result)) {
                batch_results.push_back(result);
                existing_configs.insert(q_local);
                valid_config_found = true;
            } else {
                // Configuration is unique but failed processing (collision,
                // etc.) Still mark it as seen to avoid infinite loops
                existing_configs.insert(q_local);
                attempts++;
            }
        }
    }

    return batch_results;
}

/**
 * @brief Main workspace generation function
 *
 * **Generation Process:**
 * 1. **Initialization**: Set up hash set for duplicate detection and result
 * storage
 * 2. **Batch Processing**: Process configurations in batches for efficiency
 * 3. **Duplicate Handling**: Retry with new random values for duplicates
 * 4. **Progress Reporting**: Display progress at regular intervals
 * 5. **Statistics**: Print generation statistics and performance metrics
 *
 * **Performance Characteristics:**
 * - Time complexity: O(num_points * max_attempts_per_sample)
 * - Space complexity: O(successful_samples)
 * - Typical success rate: 10-30% depending on robot geometry
 * - Batch processing improves cache locality
 *
 * **Progress Reporting:**
 * - Reports progress every 1% of total configurations
 * - Shows percentage, count, and processing rate
 * - Provides real-time feedback for long-running operations
 *
 * **Output Quality:**
 * - All returned configurations are collision-free
 * - All configurations respect ground clearance
 * - All configurations are unique within tolerance
 * - Configurations are distributed across reachable workspace
 *
 * **Memory Management:**
 * - Pre-allocates result vector with conservative estimate
 * - Uses hash set for efficient duplicate detection
 * - Minimal memory allocation during processing
 * - Efficient memory usage for large datasets
 */
std::vector<WorkspacePoint> ReachableWorkspaceGenerator::generateWorkspace() {
    auto start_time = std::chrono::high_resolution_clock::now();

    std::cout << "Generating " << num_points_
              << " random joint configurations..." << std::endl;

    // Hash set for O(1) duplicate checking
    std::unordered_set<JointConfig, JointConfigHash, JointConfigEqual>
        existing_configs;
    std::vector<WorkspacePoint> all_workspace_points;
    all_workspace_points.reserve(num_points_ / 10); // Conservative estimate

    long unsigned int processed_count = 0;

    // Progress reporting interval (every 1% of total)
    const long unsigned int progress_interval =
        std::max(1UL, num_points_ / 100);

    std::cout << "Starting optimized batch processing with duplicate retry..."
              << std::endl;

    // Process in batches for better cache locality
    for (long unsigned int batch_start = 0; batch_start < num_points_;
         batch_start += batch_size_) {
        int current_batch_size =
            std::min(static_cast<int>(batch_size_),
                     static_cast<int>(num_points_ - batch_start));

        // Process current batch
        auto batch_results = processBatch(random_generator_, existing_configs,
                                          current_batch_size);

        // Add batch results to overall collection
        all_workspace_points.insert(all_workspace_points.end(),
                                    batch_results.begin(), batch_results.end());

        processed_count += current_batch_size;

        // Progress reporting
        if (processed_count % progress_interval < current_batch_size) {
            std::cout << "Progress: " << (processed_count * 100 / num_points_)
                      << "% (" << processed_count << "/" << num_points_ << ")"
                      << std::endl;
        }
    }
    std::cout << std::endl;

    // Calculate and display generation statistics
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

    std::cout << "\nOptimized processing completed in " << duration.count()
              << " seconds" << std::endl;
    std::cout << "Generated " << all_workspace_points.size()
              << " workspace points." << std::endl;
    std::cout << "Total unique joint configurations: "
              << existing_configs.size() << std::endl;
    std::cout << "Processing rate: " << (num_points_ / duration.count())
              << " samples/second" << std::endl;
    std::cout << "Success rate: "
              << (all_workspace_points.size() * 100.0 / num_points_) << "% ("
              << all_workspace_points.size() << "/" << num_points_ << ")"
              << std::endl;
    std::cout
        << "Note: Duplicate configurations were retried with new random values"
        << std::endl;

    // Example: Print first few entries (commented out for performance)
    // std::cout << "\nFirst 5 workspace points:" << std::endl;
    // int count = 0;
    // for (const auto &point : all_workspace_points) {
    //     if (count >= 5)
    //         break;
    //     std::cout << "Position: " << point.position.transpose()
    //               << " | Rotation (quat): [" << point.rotation.w() << ", "
    //               << point.rotation.x() << ", " << point.rotation.y() << ", "
    //               << point.rotation.z() << "]"
    //               << " | Joint config: " << point.joint_config.transpose()
    //               << std::endl;
    //     count++;
    // }

    return all_workspace_points;
}

/**
 * @brief Generate and save workspace data in multiple formats with timestamped
 * filenames
 *
 * **Output Files Generated:**
 * 1. **Binary PLY**: Optimized for visualization software
 * 2. **ASCII PLY**: Human-readable format for debugging
 * 3. **K-d Tree Data**: Binary format for spatial indexing
 * 4. **Voxel Grid**: Sparse grid for fast neighbor search
 *
 * **File Naming Convention:**
 * - All files use timestamp format: YYYYMMDD_HHMMSS
 * - Ensures unique filenames across multiple runs
 * - Enables automatic loading of latest data
 *
 * **Directory Handling:**
 * - Creates output directory if it doesn't exist
 * - Provides detailed feedback on file creation
 * - Reports file sizes and memory usage
 *
 * **Cross-Platform Compatibility:**
 * - Uses std::filesystem for directory operations
 * - Handles different path separators automatically
 * - Works on Windows, Linux, and macOS
 *
 * **Timestamp Generation:**
 * - Uses system clock for current time
 * - Cross-platform localtime functions
 * - Consistent format across all files
 */
void ReachableWorkspaceGenerator::generateAndSaveWorkspace(
    const std::string &output_dir) {
    // Generate the workspace
    std::cout << "Generating and saving workspace..." << std::endl;
    auto workspace_points = generateWorkspace();

    // Generate timestamp once for all files
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &now_c);
#else
    localtime_r(&now_c, &tm_buf);
#endif
    std::ostringstream timestamp_oss;
    timestamp_oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    std::string timestamp = timestamp_oss.str();

    // Create output directory if it doesn't exist
    if (!std::filesystem::exists(output_dir)) {
        std::filesystem::create_directory(output_dir);
    }

    // Save workspace points to binary PLY file
    std::string ply_filename =
        output_dir + "/workspace_points_optimized_" + timestamp + ".ply";
    writeBinaryPLY(ply_filename, workspace_points);

    // Save workspace points to ASCII PLY file (human-readable)
    std::string ascii_ply_filename =
        output_dir + "/workspace_points_ascii_" + timestamp + ".ply";
    writeAsciiPLY(ascii_ply_filename, workspace_points);

    // Save k-d tree data to binary file
    std::string kdtree_filename =
        output_dir + "/kdtree_data_optimized_" + timestamp + ".bin";
    writeKDTreeData(kdtree_filename, workspace_points);

    // Build and save 3D voxel grid
    ReachableWorkspaceGenerator::buildAndSaveVoxelGrid(workspace_points,
                                                       output_dir, timestamp);

    std::cout << "Workspace generation and saving completed successfully!"
              << std::endl;
    std::cout << "All files saved to: " << output_dir << std::endl;
    std::cout << "Timestamp used: " << timestamp << std::endl;
}

/**
 * @brief Write workspace points to binary PLY file for visualization
 *
 * **PLY Format:**
 * - Binary little-endian format for efficiency
 * - Includes position (x, y, z) and joint configuration (6 values)
 * - Compatible with visualization software (MeshLab, CloudCompare, etc.)
 *
 * **File Structure:**
 * - Header with format and element information
 * - Binary data for each workspace point
 * - Optimized for fast loading in visualization tools
 *
 * **Header Information:**
 * - Format: binary_little_endian 1.0
 * - Element count: number of workspace points
 * - Properties: x, y, z (position) + joint1-joint6 (joint angles)
 *
 * **Binary Data:**
 * - Each point: 9 float values (3 position + 6 joint angles)
 * - Little-endian format for cross-platform compatibility
 * - Efficient storage and loading
 *
 * **Usage:**
 * - Load in 3D visualization software
 * - Analyze workspace coverage and distribution
 * - Debug workspace generation issues
 * - Create visual representations of robot workspace
 */
void ReachableWorkspaceGenerator::writeBinaryPLY(
    const std::string &filename,
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

    // Write binary data for each workspace point
    for (const auto &point : workspace_points) {
        // Convert position to float for PLY format
        float x = static_cast<float>(point.position[0]);
        float y = static_cast<float>(point.position[1]);
        float z = static_cast<float>(point.position[2]);
        ply_file.write(reinterpret_cast<const char *>(&x), sizeof(float));
        ply_file.write(reinterpret_cast<const char *>(&y), sizeof(float));
        ply_file.write(reinterpret_cast<const char *>(&z), sizeof(float));

        // Write joint configuration values as floats
        for (int j = 0; j < point.joint_config.size(); ++j) {
            float joint_val = static_cast<float>(point.joint_config[j]);
            ply_file.write(reinterpret_cast<const char *>(&joint_val),
                           sizeof(float));
        }
    }

    ply_file.close();
    std::cout << "Workspace points saved to: " << filename << std::endl;
}

/**
 * @brief Write workspace points to ASCII PLY file for human readability
 *
 * **PLY Format:**
 * - ASCII format for easy inspection and debugging
 * - Includes position (x, y, z) and joint configuration (6 values)
 * - Human-readable for manual verification
 *
 * **File Structure:**
 * - Header with format and element information
 * - ASCII data for each workspace point
 * - Space-separated values for easy parsing
 *
 * **Use Cases:**
 * - Debug workspace generation issues
 * - Manual inspection of workspace data
 * - Documentation and analysis
 * - Small datasets where readability is important
 * - Script-based processing and analysis
 *
 * **Data Format:**
 * - Each line: x y z joint1 joint2 joint3 joint4 joint5 joint6
 * - Space-separated values
 * - Float precision for numerical accuracy
 * - Easy to parse with text processing tools
 */
void ReachableWorkspaceGenerator::writeAsciiPLY(
    const std::string &filename,
    const std::vector<WorkspacePoint> &workspace_points) {
    std::ofstream ply_file(filename, std::ios::out);

    if (!ply_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename
                  << " for writing." << std::endl;
        return;
    }

    // Write PLY header
    ply_file << "ply" << std::endl;
    ply_file << "format ascii 1.0" << std::endl;
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

    // Write ASCII data for each workspace point
    for (const auto &point : workspace_points) {
        // Convert position to float for PLY format
        float x = static_cast<float>(point.position[0]);
        float y = static_cast<float>(point.position[1]);
        float z = static_cast<float>(point.position[2]);

        // Write position coordinates
        ply_file << x << " " << y << " " << z;

        // Write joint configuration values
        for (int j = 0; j < point.joint_config.size(); ++j) {
            float joint_val = static_cast<float>(point.joint_config[j]);
            ply_file << " " << joint_val;
        }

        ply_file << std::endl;
    }

    ply_file.close();
    std::cout << "Workspace points saved to ASCII PLY: " << filename
              << std::endl;
}

/**
 * @brief Write workspace points to binary k-d tree data file
 *
 * **File Format:**
 * - Header: number of points (size_t)
 * - For each point: position (3×double), quaternion (4×double), joints
 * (6×double)
 * - Optimized for fast loading in IK solver
 *
 * **Data Structure:**
 * - Position: 3D Cartesian coordinates in meters
 * - Quaternion: Unit quaternion for rotation (w, x, y, z)
 * - Joint configuration: 6 joint angles in radians
 *
 * **Binary Format:**
 * - Uses double precision for numerical accuracy
 * - Sequential storage for efficient I/O
 * - No compression for maximum loading speed
 * - Cross-platform binary compatibility
 *
 * **Usage:**
 * - Loaded by IK solver for nearest neighbor search
 * - Enables fast warm-start initialization
 * - Provides complete pose and configuration mapping
 * - Used for spatial indexing and neighbor search
 *
 * **Performance:**
 * - Fast binary I/O operations
 * - Minimal parsing overhead
 * - Efficient memory usage
 * - Suitable for real-time applications
 */
void ReachableWorkspaceGenerator::writeKDTreeData(
    const std::string &filename,
    const std::vector<WorkspacePoint> &workspace_points) {
    std::ofstream kdtree_file(filename, std::ios::binary);

    if (!kdtree_file.is_open()) {
        std::cerr << "Error: Could not open file " << filename
                  << " for writing." << std::endl;
        return;
    }

    // Write header information (number of points)
    size_t num_points = workspace_points.size();
    kdtree_file.write(reinterpret_cast<const char *>(&num_points),
                      sizeof(size_t));

    // Write data for each workspace point
    for (const auto &point : workspace_points) {
        // Write position (3 doubles: x, y, z)
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

/**
 * @brief Build and save sparse voxel grid for spatial indexing
 *
 * **Voxel Grid Algorithm:**
 * 1. **Bounding Box**: Compute axis-aligned bounding box of workspace
 * 2. **Grid Creation**: Create regular 3D grid with 1cm resolution
 * 3. **Sparse Storage**: Store only occupied cells to save memory
 * 4. **BFS Fill**: Use breadth-first search to fill empty cells
 * 5. **Serialization**: Save grid data in binary format
 *
 * **Grid Properties:**
 * - Resolution: 1cm voxel size (configurable)
 * - Sparse storage: Only occupied cells stored
 * - BFS fill: Empty cells inherit nearest neighbor
 * - Memory efficient: O(occupied_cells) vs O(total_cells)
 *
 * **Performance Benefits:**
 * - O(1) spatial indexing for neighbor search
 * - Reduces search complexity from O(n) to O(k)
 * - Enables real-time IK solving with warm-start
 * - Memory efficient for large workspaces
 *
 * **BFS Fill Strategy:**
 * - Starts from occupied cells (distance 0)
 * - Expands to 6-connected neighbors
 * - Inherits nearest point index from source
 * - Continues until max_distance reached
 *
 * **File Format:**
 * - Header: minB(3×double), r(double), dims(3×int), maxDistance(int),
 * numCells(size_t)
 * - Data: (cell_index, nearest_point_index) pairs
 * - Binary format for fast loading
 */
void ReachableWorkspaceGenerator::buildAndSaveVoxelGrid(
    const std::vector<WorkspacePoint> &workspace_points,
    const std::string &output_dir, const std::string &timestamp) {
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

    // Assign each workspace point to its corresponding grid cell
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
    int maxDistance = 5; // Default max distance for static version

    // Seed the queue with every non-empty cell
    for (const auto &cell : sparseGrid) {
        sparseNearestIdx[cell.first] = static_cast<int>(
            cell.second[0]);     // Pick first sample as representative
        q.push({cell.first, 0}); // Distance 0 for actual workspace cells
    }

    // BFS flood-fill with distance limit
    // 6-connected neighbors: (±1,0,0), (0,±1,0), (0,0,±1)
    const int offs[6][3] = {{+1, 0, 0}, {-1, 0, 0}, {0, +1, 0},
                            {0, -1, 0}, {0, 0, +1}, {0, 0, -1}};

    while (!q.empty()) {
        auto [c, distance] = q.front();
        q.pop();

        // Stop if we've reached the maximum distance
        if (distance >= maxDistance) {
            continue;
        }

        // Decode c → (x,y,z) grid coordinates
        int x = static_cast<int>(c / (dims.y() * dims.z()));
        int y = static_cast<int>((c / dims.z()) % dims.y());
        int z = static_cast<int>(c % dims.z());

        // Check all 6-connected neighbors
        for (const auto &o : offs) {
            int nx = x + o[0], ny = y + o[1], nz = z + o[2];
            if (nx < 0 || ny < 0 || nz < 0 || nx >= dims.x() ||
                ny >= dims.y() || nz >= dims.z()) {
                continue; // Skip out-of-bounds neighbors
            }

            // Compute flat index for neighbor cell
            size_t nc = static_cast<size_t>(nx) * dims.y() * dims.z() +
                        static_cast<size_t>(ny) * dims.z() +
                        static_cast<size_t>(nz);

            // If neighbor cell not yet filled, inherit nearest point and
            // continue BFS
            if (sparseNearestIdx.find(nc) == sparseNearestIdx.end()) {
                // Inherit the same representative point from source cell
                sparseNearestIdx[nc] = sparseNearestIdx[c];
                q.push({nc, distance + 1});
            }
        }
    }

    // Count filled cells after BFS
    size_t filledCells = sparseNearestIdx.size();
    std::cout << "Filled cells after limited BFS: " << filledCells << std::endl;

    // D. Serialize sparse data for runtime
    std::string grid_filename =
        output_dir + "/workspace_grid_sparse_" + timestamp + ".bin";
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

/**
 * @brief Generate timestamped filename with specified prefix and extension
 *
 * **Filename Format:**
 * - Pattern: base_dir/prefix_YYYYMMDD_HHMMSS.extension
 * - Timestamp: Current system time in YYYYMMDD_HHMMSS format
 * - Ensures unique filenames across multiple runs
 *
 * **Directory Handling:**
 * - Creates base directory if it doesn't exist
 * - Uses cross-platform directory creation
 * - Handles path separators automatically
 *
 * **Cross-Platform Compatibility:**
 * - Uses std::filesystem for directory operations
 * - Handles different path separators automatically
 * - Works on Windows, Linux, and macOS
 *
 * **Timestamp Generation:**
 * - Uses system clock for current time
 * - Cross-platform localtime functions (_WIN32 vs POSIX)
 * - Consistent format across all files
 *
 * **Usage:**
 * - Generate unique filenames for workspace data
 * - Enable automatic loading of latest data
 * - Prevent file overwrites in batch processing
 * - Ensure reproducible file naming
 */
std::string ReachableWorkspaceGenerator::getTimestampedFilename(
    const std::string &base_dir, const std::string &prefix,
    const std::string &extension) {
    if (!std::filesystem::exists(base_dir)) {
        std::filesystem::create_directory(base_dir);
    }

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
