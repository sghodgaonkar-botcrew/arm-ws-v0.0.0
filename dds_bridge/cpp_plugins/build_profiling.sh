#!/bin/bash

# IK System Profiling Build Script
# This script builds the profiling example with proper optimization flags

set -e  # Exit on any error

echo "=== IK System Profiling Build Script ==="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    print_error "CMakeLists.txt not found. Please run this script from the cpp_plugins directory."
    exit 1
fi

# Create build directory
print_status "Creating build directory..."
rm -rf build
mkdir -p build
cd build

# Configure with CMake
print_status "Configuring with CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-O3 -march=native -DENABLE_PROFILING" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Build the project
print_status "Building project..."
make -j$(nproc)

# Check if build was successful
if [ -f "profile_example" ]; then
    print_status "Build successful! Executable created: profile_example"
else
    print_error "Build failed! Executable not found."
    exit 1
fi

# Check for URDF file
URDF_PATH="../../urdf/arm_v0.1/arm.urdf"
if [ ! -f "$URDF_PATH" ]; then
    print_warning "URDF file not found at $URDF_PATH"
    print_warning "You may need to update the path in profile_example.cpp"
else
    print_status "URDF file found at $URDF_PATH"
fi

# Run the profiling example
print_status "Running profiling example..."
print_status "This will test various IK operations and generate performance reports..."
echo

# Run with timeout to prevent hanging
timeout 60s ./profile_example || {
    print_warning "Program timed out or encountered an error"
    print_warning "This might be due to missing dependencies or URDF file issues"
}

echo
print_status "Build and test completed!"
print_status "Check the output above for performance analysis."
print_status "For detailed usage instructions, see PROFILING_README.md"

# Optional: Show system information
echo
print_status "System Information:"
echo "CPU: $(lscpu | grep 'Model name' | cut -f 2 -d ":")"
echo "Memory: $(free -h | grep Mem | awk '{print $2}')"
echo "Compiler: $(gcc --version | head -n1)"
echo "Build type: Release with -O3 optimization" 