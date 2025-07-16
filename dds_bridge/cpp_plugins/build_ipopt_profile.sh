#!/bin/bash

echo "=== Building IPOPT Profiling Test ==="

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo "CMake configuration failed!"
    exit 1
fi

# Build the project
echo "Building project..."
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# Check if IPOPT profiling test was built
if [ -f "ipopt_profile_test" ]; then
    echo "IPOPT profiling test built successfully!"
    echo ""
    echo "=== Running IPOPT Profiling Test ==="
    echo "This will test IPOPT solver performance with detailed timing..."
    echo ""
    
    # Run the IPOPT profiling test
    ./ipopt_profile_test
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "=== IPOPT Profiling Test Completed Successfully ==="
        echo "Check the output above for detailed performance analysis."
    else
        echo ""
        echo "=== IPOPT Profiling Test Failed ==="
        echo "The test encountered an error. Check the output above for details."
    fi
else
    echo "IPOPT profiling test was not built (IPOPT may not be available)"
    echo "Available executables:"
    ls -la *.exe 2>/dev/null || ls -la profile_example 2>/dev/null || echo "No executables found"
fi

echo ""
echo "=== Build Summary ==="
echo "Build directory: $(pwd)"
echo "Available targets:"
if [ -f "profile_example" ]; then
    echo "  - profile_example (basic profiling)"
fi
if [ -f "ipopt_profile_test" ]; then
    echo "  - ipopt_profile_test (IPOPT-specific profiling)"
fi 