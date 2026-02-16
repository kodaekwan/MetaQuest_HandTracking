#!/bin/bash
# Build script for ZED Camera Manager
# Run inside Docker container

set -e

echo "=== ZED Camera Manager Build Script ==="

# Check if inside Docker
if [ ! -f /.dockerenv ] && [ ! -d /usr/local/zed ]; then
    echo "Warning: This script is designed to run inside the ZED Docker container"
    echo "Consider using: docker run ... stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04"
fi

# Install dependencies if needed
echo "[1/3] Checking dependencies..."
if ! pkg-config --exists zbar 2>/dev/null; then
    echo "Installing zbar..."
    apt-get update && apt-get install -y libzbar-dev
fi

if ! pkg-config --exists opencv4 2>/dev/null; then
    echo "Installing OpenCV..."
    apt-get update && apt-get install -y libopencv-dev
fi

# Build with CMake
echo "[2/3] Building with CMake..."
mkdir -p build
cd build
cmake ..
make -j$(nproc)

# Copy executable to project root
echo "[3/3] Copying executable..."
cp zed_camera_manager ..
cd ..

echo ""
echo "=== Build Complete ==="
echo "Run: ./zed_camera_manager"
echo "Or:  ./zed_camera_manager --help"
