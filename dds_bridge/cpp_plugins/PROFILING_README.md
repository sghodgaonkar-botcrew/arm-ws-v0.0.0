# IK System Profiling Guide

This guide explains how to use the profiling functionality added to the IK (Inverse Kinematics) system to identify performance bottlenecks and optimize your robot control application.

## Overview

The profiling system has been integrated into the following components:
- `IKModel` - Core kinematics and cost function computations
- `IkIpoptSolver` - IPOPT-based optimization solver
- Example program demonstrating profiling usage

## Features

### 1. High-Resolution Timing
- Uses `std::chrono::high_resolution_clock` for microsecond precision
- Thread-safe data collection with mutex protection
- Minimal overhead when profiling is disabled

### 2. Comprehensive Coverage
The profiling system measures execution time for:

#### IKModel Operations:
- **Constructor operations**: URDF loading, model creation, frame search
- **Forward kinematics**: Pinocchio FK computation, frame placement updates
- **Cost function evaluation**: Error computation, matrix operations
- **Gradient computation**: Jacobian calculation, error transforms, matrix multiplications
- **Hessian computation**: Full Hessian matrix evaluation
- **Warm start optimization**: Iterative position-only IK
- **Coordinate transformations**: SE3 ↔ XYZQuat conversions

#### IK Solver Operations:
- **Solver initialization**: Warm start, bounds setup
- **IPOPT callbacks**: Cost evaluation, gradient computation, Hessian evaluation
- **Matrix operations**: Sparsity structure, matrix factorizations

### 3. Detailed Statistics
For each profiled operation, the system tracks:
- **Count**: Number of times the operation was called
- **Total time**: Cumulative execution time in milliseconds
- **Average time**: Mean execution time in microseconds
- **Minimum time**: Fastest execution time
- **Maximum time**: Slowest execution time

## Usage

### 1. Enable Profiling

To enable profiling, define the `ENABLE_PROFILING` macro before including the headers:

```cpp
#define ENABLE_PROFILING
#include "ik_model.h"
#include "ik_solver_v2.hpp"
```

Or compile with the flag:
```bash
g++ -DENABLE_PROFILING your_program.cpp
```

### 2. Basic Usage Example

```cpp
#include "ik_model.h"
#include "ik_solver_v2.hpp"

int main() {
    // Reset any existing profile data
    IKModel::resetProfileData();
    IkIpoptSolver::resetProfileData();
    
    // Create IK model
    IKModel ik_model("robot.urdf", "end_effector");
    
    // Perform your IK operations...
    JointConfig q = JointConfig::Random();
    auto pose = ik_model.computeForwardKinematics(q);
    
    // Print performance report
    IKModel::printProfileReport();
    IkIpoptSolver::printProfileReport();
    
    return 0;
}
```

### 3. Running the Profiling Example

Build and run the included example:

```bash
cd dds_bridge/cpp_plugins
mkdir build && cd build
cmake ..
make
./profile_example
```

## Sample Output

The profiling system generates detailed reports like this:

```
=== PERFORMANCE PROFILE REPORT ===
Function/Operation           Count          Total (ms)    Avg (μs)       Min (μs)       Max (μs)
---------------------------------------------------------------------------------------------------------
forward_kinematics          100            45.23         452.30         398.12         521.45
jacobian_computation        50             23.45         468.90         412.33         589.12
cost_function               50             12.34         246.80         198.45         345.67
cost_gradient               50             34.56         691.20         623.45         789.12
warm_start_optimization     5              156.78        31356.00       28912.34       34567.89
```

## Identifying Bottlenecks

### 1. High-Frequency Operations
Look for operations that are called frequently with high average times:
- **Forward kinematics**: Should be < 500μs for 6-DOF robots
- **Jacobian computation**: Should be < 1000μs
- **Cost function evaluation**: Should be < 300μs

### 2. Optimization Bottlenecks
- **Warm start optimization**: Can take 10-50ms depending on convergence
- **IPOPT solver iterations**: Each iteration involves multiple function evaluations
- **Hessian computation**: Most expensive operation in optimization

### 3. Common Performance Issues

#### Pinocchio Operations
- **Forward kinematics**: Bottleneck in `pinocchio::forwardKinematics()`
- **Jacobian computation**: Expensive in `pinocchio::computeFrameJacobian()`
- **Frame updates**: `pinocchio::updateFramePlacements()` can be optimized

#### Matrix Operations
- **Matrix multiplications**: Large matrices in gradient/Hessian computation
- **Matrix inversions**: Pseudo-inverse in warm start optimization
- **Eigen operations**: Quaternion conversions and transformations

#### Memory Allocation
- **Temporary objects**: Frequent creation of Eigen matrices
- **Dynamic allocations**: In Jacobian and Hessian computations

## Optimization Strategies

### 1. Algorithmic Optimizations
- **Caching**: Store frequently computed values (Jacobians, transforms)
- **Early termination**: Stop warm start when close enough to target
- **Adaptive tolerances**: Use looser tolerances for initial iterations

### 2. Implementation Optimizations
- **Memory pooling**: Reuse matrix objects instead of allocating new ones
- **SIMD optimization**: Ensure Eigen uses vectorized operations
- **Compile-time optimizations**: Use `-O3` and enable vectorization

### 3. System-Level Optimizations
- **CPU affinity**: Pin threads to specific CPU cores
- **Memory layout**: Optimize data structure alignment
- **Cache utilization**: Minimize cache misses in matrix operations

## Advanced Usage

### 1. Custom Profiling Points

Add custom profiling points in your code:

```cpp
PROFILE_START(my_custom_operation);
// Your expensive operation here
PROFILE_END(my_custom_operation);
```

### 2. Conditional Profiling

Profile only specific sections:

```cpp
#ifdef ENABLE_PROFILING
    PROFILE_START(expensive_section);
#endif

// Your code here

#ifdef ENABLE_PROFILING
    PROFILE_END(expensive_section);
#endif
```

### 3. Thread-Safe Profiling

The profiling system is thread-safe, so you can use it in multi-threaded applications:

```cpp
// Thread 1
IKModel::addProfileData("thread1_operation", duration);

// Thread 2  
IKModel::addProfileData("thread2_operation", duration);

// Both threads can safely call printProfileReport()
```

## Troubleshooting

### 1. Compilation Issues
- Ensure `ENABLE_PROFILING` is defined
- Check that all required headers are included
- Verify C++17 standard is enabled

### 2. Runtime Issues
- Profile data is thread-safe but may show interleaved results
- Large profile datasets may consume significant memory
- Consider calling `resetProfileData()` periodically in long-running applications

### 3. Performance Impact
- Profiling adds minimal overhead when enabled
- Disable profiling in production for maximum performance
- Use sampling-based profiling for very high-frequency operations

## Integration with Other Tools

### 1. gprof Integration
Combine with gprof for system-level profiling:
```bash
g++ -pg -DENABLE_PROFILING your_program.cpp
./your_program
gprof your_program gmon.out > profile.txt
```

### 2. perf Integration
Use Linux perf for hardware-level profiling:
```bash
perf record ./your_program
perf report
```

### 3. Valgrind Integration
Profile memory usage with Valgrind:
```bash
valgrind --tool=callgrind ./your_program
kcachegrind callgrind.out.*
```

## Conclusion

The profiling system provides detailed insights into the performance characteristics of your IK system. Use the generated reports to identify bottlenecks and optimize the most critical operations for your specific use case.

For questions or issues, refer to the source code comments or create an issue in the project repository. 