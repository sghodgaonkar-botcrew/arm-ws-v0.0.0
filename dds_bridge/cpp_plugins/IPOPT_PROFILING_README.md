# IPOPT Solver Profiling Guide

This guide explains how to profile the IPOPT solver performance in the IK system to identify bottlenecks and optimize performance.

## Overview

The IPOPT (Interior Point OPTimizer) solver is typically the most computationally expensive component of the inverse kinematics system. This profiling setup helps you:

1. **Identify bottlenecks** in the IPOPT optimization process
2. **Measure timing** of different IPOPT operations
3. **Analyze function evaluation costs** (objective, gradient, Hessian)
4. **Optimize solver parameters** based on performance data
5. **Compare different configurations** and approaches

## Available Profiling Tools

### 1. Basic Profiling (`profile_example`)
- Tests core IK operations (forward kinematics, cost, gradient, Hessian)
- Includes basic IPOPT testing (if available)
- Good for overall system performance analysis

### 2. IPOPT-Specific Profiling (`ipopt_profile_test`)
- Focuses specifically on IPOPT solver performance
- Detailed timing of IPOPT operations
- Handles crashes gracefully to ensure profiling data is collected

### 3. Detailed IPOPT Analysis (`ipopt_detailed_profile`)
- Comprehensive IPOPT performance analysis
- Per-operation timing breakdown
- Multiple test runs for statistical analysis
- Performance recommendations

## Building and Running

### Prerequisites
- IPOPT library installed and configured
- Pinocchio library
- Eigen3 library
- CMake 3.16+

### Build Commands

```bash
# Build all profiling tools
./build_ipopt_profile.sh

# Or build manually
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Running the Profiling Tools

```bash
# Basic profiling
./profile_example

# IPOPT-specific profiling
./ipopt_profile_test

# Detailed IPOPT analysis
./ipopt_detailed_profile
```

## Understanding the Profiling Output

### Key Metrics to Monitor

1. **Solver Creation Time**
   - Time to create the IK solver instance
   - Should be minimal for repeated use

2. **IPOPT Application Setup**
   - Time to create and configure IPOPT application
   - Includes parameter setting and initialization

3. **Optimization Time**
   - Total time for the optimization process
   - Most important metric for performance

4. **Function Evaluation Times**
   - Objective function calls (cost computation)
   - Gradient function calls (gradient computation)
   - Hessian function calls (Hessian computation)

5. **Per-Iteration Time**
   - Average time per optimization iteration
   - Helps identify if the bottleneck is in iterations or setup

### Sample Output Interpretation

```
=== DETAILED IPOPT TIMING ANALYSIS ===
Operation                                Calls     Total (ms)    Avg (μs)      Min (μs)      Max (μs)
Solver Creation                          3         0.15          50.00         45            58
IPOPT App Creation                       3         0.08          26.67         22            35
IPOPT Configuration                      3         0.12          40.00         35            48
IPOPT Initialization                     3         1.25          416.67        380           450
IPOPT Optimization                       3         45.67         15223.33      12000         18000
```

**Analysis:**
- **Solver Creation**: Fast (~50μs) - not a bottleneck
- **IPOPT Setup**: Moderate (~500μs total) - acceptable
- **Optimization**: Slow (~15ms average) - this is the bottleneck
- **Variability**: High max/min ratio suggests inconsistent performance

## Performance Optimization Strategies

### 1. Reduce Function Evaluation Overhead
- Profile the `cost`, `gradient`, and `Hessian` functions
- Look for expensive operations in forward kinematics
- Consider caching intermediate results

### 2. Optimize IPOPT Parameters
```cpp
// Recommended settings for performance
app->Options()->SetIntegerValue("print_level", 0);        // Disable output
app->Options()->SetStringValue("sb", "yes");              // Suppress banner
app->Options()->SetNumericValue("tol", 1e-6);            // Relax tolerance
app->Options()->SetIntegerValue("max_iter", 100);        // Limit iterations
app->Options()->SetStringValue("linear_solver", "mumps"); // Fast linear solver
```

### 3. Use Warm Starts
- Initialize with good starting configurations
- Reuse previous solutions when possible
- Implement solution caching

### 4. Parallel Processing
- Run multiple IK problems in parallel
- Use thread-safe implementations
- Consider GPU acceleration for function evaluations

## Common Bottlenecks and Solutions

### High Optimization Time
- **Cause**: Too many iterations or expensive function evaluations
- **Solution**: Better initial guesses, relaxed tolerances, function optimization

### High Function Evaluation Time
- **Cause**: Expensive forward kinematics or matrix operations
- **Solution**: Cache results, optimize algorithms, use approximations

### High Setup Time
- **Cause**: Repeated IPOPT application creation
- **Solution**: Reuse IPOPT application instances

### Memory Issues
- **Cause**: Large matrices or excessive allocations
- **Solution**: Use memory pools, reduce matrix copies, optimize data structures

## Troubleshooting

### Segmentation Faults
- Common with IPOPT due to complex memory management
- Use the profiling tools with error handling
- Check IPOPT version compatibility

### Missing IPOPT
- Install IPOPT: `sudo apt-get install libipopt-dev`
- Or build from source with proper linking
- Verify with `pkg-config --exists ipopt`

### Poor Performance
- Check if profiling shows expected bottlenecks
- Verify IPOPT parameters are appropriate
- Consider alternative solvers (SNOPT, KNITRO)

## Advanced Profiling

### Custom Profiling Points
Add your own profiling points in the code:

```cpp
PROFILE_START(my_operation);
// ... your code ...
PROFILE_END(my_operation);
```

### Memory Profiling
Use tools like Valgrind or AddressSanitizer:

```bash
valgrind --tool=massif ./ipopt_detailed_profile
```

### CPU Profiling
Use gprof or perf:

```bash
gprof ./ipopt_detailed_profile gmon.out > profile.txt
perf record ./ipopt_detailed_profile
perf report
```

## Conclusion

The IPOPT profiling tools provide detailed insights into solver performance. Focus on:

1. **Optimization time** - the main bottleneck
2. **Function evaluation costs** - where most time is spent
3. **Parameter tuning** - for your specific problem
4. **Warm starts** - to reduce iteration count

Use the profiling data to guide optimization efforts and achieve the best performance for your IK system. 