#!/bin/bash

# Script to update Doxygen documentation for C++ Plugins
# Usage: ./update_docs.sh

echo "Updating Doxygen documentation for C++ Plugins..."

# Check if doxygen is installed
if ! command -v doxygen &> /dev/null; then
    echo "Error: doxygen is not installed. Please install it first:"
    echo "sudo apt install doxygen"
    exit 1
fi

# Navigate to docs directory
cd "$(dirname "$0")"

# Clean previous documentation
echo "Cleaning previous documentation..."
rm -rf html/* latex/*

# Generate new documentation
echo "Generating new documentation..."
doxygen Doxyfile

# Check if generation was successful
if [ $? -eq 0 ]; then
    echo "Documentation generated successfully!"
    echo "HTML documentation is available in: $(pwd)/html/index.html"
    echo "LaTeX documentation is available in: $(pwd)/latex/"
    
    # Count the number of documented classes
    class_count=$(grep -c "class.*\.html\|struct.*\.html" html/classes.html 2>/dev/null || echo "0")
    echo "Documented classes and structures: $class_count"
    
    # List the main classes
    echo "Main classes documented:"
    echo "- IKSolverWithRWS"
    echo "- RobotModel" 
    echo "- ReachableWorkspaceGenerator"
    echo "- MotorController"
    echo "- WorkspacePoint"
    echo "- NearestNeighbor"
    echo "- VoxelGrid"
    echo "- WorkspaceAdaptor"
    echo "- JointConfigHash"
    echo "- JointConfigEqual"
    
else
    echo "Error: Documentation generation failed!"
    exit 1
fi 