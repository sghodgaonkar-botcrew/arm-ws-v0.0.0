#include "reachable_workspace_generator.h"
#include "robot_model.h"
#include <iostream>

int main() {
    try {
        // Create IKModel instance
        RobotModel robot_model(
            "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.urdf",
            "connection_frame",
            "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/ur10.srdf");

        // Create the workspace generator with parameters passed directly
        ReachableWorkspaceGenerator generator(
            robot_model, // RobotModel reference
            50000,       // num_points
            1000,        // batch_size
            0.05,        // ground_threshold
            1e-3,        // joint_tolerance
            10           // max_attempts_per_sample
        );

        // Generate and save workspace with one function call
        generator.generateAndSaveWorkspace("generated_workspaces");

        std::cout << "Workspace generation completed successfully!"
                  << std::endl;

        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
}