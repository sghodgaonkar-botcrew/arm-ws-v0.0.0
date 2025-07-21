/**
 * @file reachable_workspace_generator.h
 * @brief Reachable Workspace Generator for Robot IK Solver
 *
 * This header file defines the ReachableWorkspaceGenerator class, which is
 * responsible for generating and sampling the reachable workspace of a robot
 * arm. The generator creates a comprehensive dataset of valid joint
 * configurations and their corresponding end-effector poses for use in IK
 * solving.
 *
 * **Key Features:**
 * - Random sampling of joint configurations within joint limits
 * - Collision detection and ground clearance validation
 * - Duplicate configuration detection and retry logic
 * - Batch processing for improved performance
 * - Multiple output formats (PLY, binary, voxel grid)
 *
 * **Generation Process:**
 * 1. Random joint configuration sampling
 * 2. Forward kinematics computation
 * 3. Collision and validity checking
 * 4. Duplicate detection and filtering
 * 5. Data storage in multiple formats
 *
 * **Output Formats:**
 * - Binary PLY files for visualization
 * - ASCII PLY files for human readability
 * - Binary k-d tree data for spatial indexing
 * - Sparse voxel grid for fast neighbor search
 *
 * @author Shantanu Ghodgaonkar
 * @date 2025-07-21
 * @version 1.0
 */

#ifndef REACHABLE_WORKSPACE_GENERATOR_H
#define REACHABLE_WORKSPACE_GENERATOR_H

#include "robot_model.h"
#include "workspace_types.h"
#include <Eigen/Geometry>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

/**
 * @brief Hash function for JointConfig to use with unordered_set (O(1) lookup)
 *
 * This struct provides a hash function for JointConfig vectors, enabling
 * efficient O(1) lookup in unordered containers. The hash function combines
 * individual joint values using a simple but effective hashing strategy.
 *
 * **Hash Strategy:**
 * - Combines hash values of individual joint angles
 * - Uses bit manipulation for good distribution
 * - Provides fast computation for real-time applications
 *
 * **Usage:**
 * - Used in unordered_set for duplicate detection
 * - Enables efficient configuration comparison
 * - Critical for performance in large workspace generation
 */
struct JointConfigHash {
    /**
     * @brief Compute hash value for a joint configuration
     * @param config Joint configuration vector to hash
     * @return Hash value for the configuration
     *
     * **Hash Algorithm:**
     * - Iterates through each joint angle
     * - Combines hash using XOR and bit shifting
     * - Uses golden ratio constant for better distribution
     * - Returns combined hash value
     */
    std::size_t operator()(const JointConfig &config) const;
};

/**
 * @brief Equality function for JointConfig to use with unordered_set
 *
 * This struct provides an equality comparison function for JointConfig vectors,
 * enabling proper duplicate detection in unordered containers. Uses
 * tolerance-based comparison to handle floating-point precision issues.
 *
 * **Comparison Strategy:**
 * - Tolerance-based comparison (1e-3 radians)
 * - Handles floating-point precision errors
 * - Ensures robust duplicate detection
 *
 * **Usage:**
 * - Used in unordered_set for duplicate detection
 * - Prevents infinite loops in workspace generation
 * - Ensures unique configuration sampling
 */
struct JointConfigEqual {
    /**
     * @brief Compare two joint configurations for equality
     * @param lhs Left-hand side joint configuration
     * @param rhs Right-hand side joint configuration
     * @return true if configurations are equal within tolerance
     *
     * **Comparison Logic:**
     * - Compares each joint angle with tolerance
     * - Returns false on first difference exceeding tolerance
     * - Handles numerical precision issues gracefully
     */
    bool operator()(const JointConfig &lhs, const JointConfig &rhs) const;
};

/**
 * @brief Main workspace generator class for robot reachable workspace sampling
 *
 * This class implements a comprehensive workspace generation system that
 * samples the reachable workspace of a robot arm by generating random joint
 * configurations and computing their corresponding end-effector poses.
 *
 * **Core Functionality:**
 * - Random sampling within joint limits
 * - Collision detection and validation
 * - Duplicate configuration handling
 * - Batch processing for efficiency
 * - Multiple output format generation
 *
 * **Generation Strategy:**
 * 1. **Sampling**: Generate random joint configurations within limits
 * 2. **Validation**: Check for collisions and ground clearance
 * 3. **Deduplication**: Remove duplicate configurations using hash set
 * 4. **Processing**: Compute forward kinematics for valid configurations
 * 5. **Output**: Save data in multiple formats for different use cases
 *
 * **Performance Optimizations:**
 * - Batch processing for better cache locality
 * - Hash-based duplicate detection (O(1) lookup)
 * - Pre-allocated vectors to avoid reallocation
 * - Progress reporting for long-running operations
 *
 * **Error Handling:**
 * - Graceful handling of invalid configurations
 * - Retry logic for duplicate configurations
 * - Comprehensive error reporting and logging
 *
 * @see RobotModel
 * @see WorkspacePoint
 * @see JointConfigHash
 * @see JointConfigEqual
 */
class ReachableWorkspaceGenerator {
  public:
    /**
     * @brief Constructor with configurable parameters
     * @param robot_model Reference to the robot model for kinematics and
     * collision checking
     * @param num_points Total number of random configurations to attempt
     * (default: 100000)
     * @param batch_size Number of configurations to process in each batch
     * (default: 1000)
     * @param ground_threshold Minimum z-coordinate for ground clearance
     * (default: 0.05)
     * @param joint_tolerance Tolerance for joint configuration comparison
     * (default: 1e-3)
     * @param max_attempts_per_sample Maximum attempts per sample before giving
     * up (default: 10)
     * @param max_distance Maximum distance for voxel grid BFS (default: 2)
     *
     * **Parameter Guidelines:**
     * - num_points: Higher values give better workspace coverage but longer
     * generation time
     * - batch_size: Larger batches improve cache locality but use more memory
     * - ground_threshold: Should match robot's base height above ground
     * - joint_tolerance: Balance between precision and performance
     * - max_attempts_per_sample: Prevents infinite loops in duplicate scenarios
     * - max_distance: Controls voxel grid fill extent for neighbor search
     *
     * **Initialization Flow:**
     * 1. Store robot model reference
     * 2. Initialize random number generator with fixed seed
     * 3. Set generation parameters
     * 4. Print initialization summary
     */
    ReachableWorkspaceGenerator(RobotModel &robot_model,
                                long unsigned int num_points = 100000,
                                int batch_size = 1000,
                                double ground_threshold = 0.05,
                                double joint_tolerance = 1e-3,
                                int max_attempts_per_sample = 10,
                                int max_distance = 50);

    /**
     * @brief Destructor
     *
     * No manual cleanup needed - RobotModel handles its own resources.
     * The destructor is virtual to ensure proper cleanup in derived classes.
     */
    ~ReachableWorkspaceGenerator();

    /**
     * @brief Main workspace generation function
     * @return Vector of valid workspace points with positions, orientations,
     * and joint configs
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
     *
     * **Output Quality:**
     * - All returned configurations are collision-free
     * - All configurations respect ground clearance
     * - All configurations are unique within tolerance
     * - Configurations are distributed across reachable workspace
     *
     * @see processBatch()
     * @see processSampleFast()
     */
    std::vector<WorkspacePoint> generateWorkspace();

    /**
     * @brief Generate and save workspace data in multiple formats with
     * timestamped filenames
     * @param output_dir Directory to save all output files
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
     * @see writeBinaryPLY()
     * @see writeAsciiPLY()
     * @see writeKDTreeData()
     * @see buildAndSaveVoxelGrid()
     */
    void generateAndSaveWorkspace(const std::string &output_dir);

    // Static utility functions (can be called without class instance)
    /**
     * @brief Write workspace points to binary PLY file for visualization
     * @param filename Output filename for binary PLY file
     * @param workspace_points Vector of workspace points to write
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
     * **Usage:**
     * - Load in 3D visualization software
     * - Analyze workspace coverage and distribution
     * - Debug workspace generation issues
     */
    static void
    writeBinaryPLY(const std::string &filename,
                   const std::vector<WorkspacePoint> &workspace_points);

    /**
     * @brief Write workspace points to ASCII PLY file for human readability
     * @param filename Output filename for ASCII PLY file
     * @param workspace_points Vector of workspace points to write
     *
     * **PLY Format:**
     * - ASCII format for easy inspection and debugging
     * - Includes position (x, y, z) and joint configuration (6 values)
     * - Human-readable for manual verification
     *
     * **Use Cases:**
     * - Debug workspace generation issues
     * - Manual inspection of workspace data
     * - Documentation and analysis
     * - Small datasets where readability is important
     */
    static void
    writeAsciiPLY(const std::string &filename,
                  const std::vector<WorkspacePoint> &workspace_points);

    /**
     * @brief Write workspace points to binary k-d tree data file
     * @param filename Output filename for binary data file
     * @param workspace_points Vector of workspace points to write
     *
     * **File Format:**
     * - Header: number of points (size_t)
     * - For each point: position (3×double), quaternion (4×double), joints
     * (6×double)
     * - Optimized for fast loading in IK solver
     *
     * **Data Structure:**
     * - Position: 3D Cartesian coordinates
     * - Quaternion: Unit quaternion for rotation (w, x, y, z)
     * - Joint configuration: 6 joint angles in radians
     *
     * **Usage:**
     * - Loaded by IK solver for nearest neighbor search
     * - Enables fast warm-start initialization
     * - Provides complete pose and configuration mapping
     */
    static void
    writeKDTreeData(const std::string &filename,
                    const std::vector<WorkspacePoint> &workspace_points);

    /**
     * @brief Build and save sparse voxel grid for spatial indexing
     * @param workspace_points Vector of workspace points to index
     * @param output_dir Directory to save voxel grid file
     * @param timestamp Timestamp string for filename
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
     *
     * @see findKNearestNeighborsVoxel()
     */
    static void
    buildAndSaveVoxelGrid(const std::vector<WorkspacePoint> &workspace_points,
                          const std::string &output_dir,
                          const std::string &timestamp);

    /**
     * @brief Generate timestamped filename with specified prefix and extension
     * @param base_dir Base directory for the file
     * @param prefix Filename prefix
     * @param extension File extension (including dot)
     * @return Complete filename with timestamp
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
     * **Usage:**
     * - Generate unique filenames for workspace data
     * - Enable automatic loading of latest data
     * - Prevent file overwrites in batch processing
     */
    static std::string getTimestampedFilename(const std::string &base_dir,
                                              const std::string &prefix,
                                              const std::string &extension);

  private:
    RobotModel &robot_model_;       ///< Reference to external RobotModel for
                                    ///< kinematics and collision checking
    std::mt19937 random_generator_; ///< Random number generator with fixed seed
                                    ///< for reproducibility

    // Workspace generation parameters
    long unsigned int
        num_points_; ///< Total number of random configurations to attempt
    int batch_size_; ///< Number of configurations to process in each batch
    double ground_threshold_; ///< Minimum z-coordinate for ground clearance
    double joint_tolerance_;  ///< Tolerance for joint configuration comparison
    int max_attempts_per_sample_; ///< Maximum attempts per sample before giving
                                  ///< up
    int max_distance_;            ///< Maximum distance for voxel grid BFS

    // Helper functions
    /**
     * @brief Process a single joint configuration and compute workspace point
     * @param q Joint configuration to process
     * @param result Output workspace point (valid only if function returns
     * true)
     * @return true if configuration is valid and workspace point was computed
     *
     * **Processing Steps:**
     * 1. **Validation**: Check if configuration is valid (collision-free, above
     * ground)
     * 2. **Forward Kinematics**: Compute end-effector pose using robot model
     * 3. **Data Extraction**: Extract position and rotation from SE3 transform
     * 4. **Quaternion Normalization**: Ensure unit quaternion for numerical
     * stability
     * 5. **Result Creation**: Create WorkspacePoint with computed data
     *
     * **Error Handling:**
     * - Returns false for invalid configurations
     * - Catches and reports exceptions during processing
     * - Provides detailed error messages for debugging
     *
     * **Performance:**
     * - Fast processing for valid configurations
     * - Early exit for invalid configurations
     * - Exception-safe implementation
     */
    bool processSampleFast(const JointConfig &q, WorkspacePoint &result);

    /**
     * @brief Process a batch of random joint configurations
     * @param gen Random number generator for configuration sampling
     * @param existing_configs Hash set of existing configurations for duplicate
     * detection
     * @param batch_size Number of configurations to process in this batch
     * @return Vector of valid workspace points from this batch
     *
     * **Batch Processing Algorithm:**
     * 1. **Random Generation**: Generate random joint configurations within
     * limits
     * 2. **Duplicate Check**: Use hash set for O(1) duplicate detection
     * 3. **Retry Logic**: Retry with new random values for duplicates
     * 4. **Validation**: Process valid configurations using processSampleFast()
     * 5. **Collection**: Collect successful workspace points
     *
     * **Performance Optimizations:**
     * - Pre-allocates result vector for efficiency
     * - Uses hash set for fast duplicate detection
     * - Limits retry attempts to prevent infinite loops
     * - Batch processing improves cache locality
     *
     * **Duplicate Handling:**
     * - Marks duplicate configurations as seen to avoid infinite loops
     * - Retries with new random values up to max_attempts_per_sample
     * - Ensures progress even with high duplicate rates
     *
     * @see processSampleFast()
     * @see JointConfigHash
     * @see JointConfigEqual
     */
    std::vector<WorkspacePoint> processBatch(
        std::mt19937 &gen,
        std::unordered_set<JointConfig, JointConfigHash, JointConfigEqual>
            &existing_configs,
        const int batch_size);
};

#endif // REACHABLE_WORKSPACE_GENERATOR_H