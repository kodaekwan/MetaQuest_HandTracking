# ZED Camera Manager Dockerfile
# Based on StereoLabs official ZED SDK image

FROM stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    libzbar-dev \
    libopencv-dev \
    python3-pip \
    python3-numpy \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --break-system-packages \
    posix_ipc \
    opencv-python \
    numpy

# Set working directory
WORKDIR /app

# Copy project files
COPY . /app/

# Build the project
RUN mkdir -p build && cd build && cmake .. && make -j$(nproc) && cp zed_camera_manager ..

# Create recordings directory
RUN mkdir -p recordings

# Expose control port
EXPOSE 12345

# Default command
CMD ["./zed_camera_manager"]
