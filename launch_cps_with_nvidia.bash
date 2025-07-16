#!/bin/bash

# Set environment variables for NVIDIA GPU rendering
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Launch CoppeliaSim with the correct path
~/CoppeliaSim_Pro_V4_10_0_rev0_Ubuntu22_04/coppeliaSim