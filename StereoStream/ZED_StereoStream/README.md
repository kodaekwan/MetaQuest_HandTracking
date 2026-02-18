# ZED Stereo Camera Manager

A high-performance C++ camera manager for ZED stereo cameras with Python interface. Features UDP streaming to Apple Vision Pro / Meta Quest, QR-based robot time synchronization, video recording with metadata, and real-time frame sharing via POSIX shared memory.

## Author

**Daekwan Ko (kodaekwan)**  
Ph.D Student  
Interactive Robotics Lab  
Dongguk University

---

## Features

### 🎥 ZED Stereo Camera
- ZED 2, ZED Mini, ZED 2i support
- Configurable resolution (VGA, HD720, HD1080, HD2K)
- Side-by-side stereo image output
- Docker-based deployment (CUDA/GPU support)

### 📡 XR Device Streaming
- **UDP streaming** to Apple Vision Pro / Meta Quest
- JPEG compression with adjustable quality
- Configurable resolution and frame rate
- Stereo parameter control (focus, quad, zoom)

### ⏱️ Time Synchronization
 - **QR-based robot time sync**: Capture QR codes with PC/Robot timestamps
 - **Microsecond precision**: All timestamps are handled in μs internally
 - **Robust parsing (timezone-aware)**: `parseTimestampToUs()` attempts multiple interpretations
     (local, UTC, and Asia/Seoul (KST)) and selects the one closest to the current system time to
     mitigate ambiguous timestamp strings coming from different hosts/timezones.
 - **Microsecond diagnostic**: The camera manager logs whether the parsed QR PC timestamp
     contains microsecond precision (✅) or appears to be only millisecond precision (⚠️).
 - **Dual interpolation methods**:
     - System time-based: Uses PC clock
     - Camera time-based: Uses ZED hardware clock (more precise)

### 📹 Video Recording
- MP4 video recording with OpenCV
- **CSV metadata** for each video:
  - Frame number, camera/system timestamps
  - QR detection status and parsed times
  - Interpolated robot time (dual methods)

### 🔗 Shared Memory Interface
- Real-time stereo frame sharing via POSIX shared memory
- Zero-copy access for Python interface
- Command interface for control without TCP

### 🐍 Python Interface
- **TCP Control**: Remote command interface
- **Shared Memory**: Direct frame access
- Combined interface for both methods

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Docker Container (CUDA)                       │
│                                                                  │
│  ┌─────────────┐      ┌──────────────────────────────────────┐  │
│  │ ZED Camera  │─────>│ zed_camera_manager (C++)             │  │
│  │  (Stereo)   │      │  • TCP Control Server (port auto)    │  │
│  └─────────────┘      │  • UDP Stereo Streaming              │  │
│                       │  • Video Recording + CSV Metadata     │  │
│                       │  • QR Recognition (ZBar)              │  │
│                       │  • Shared Memory Interface            │  │
│                       └──────────────┬───────────────────────┘  │
│                                      │                          │
│              ┌───────────────────────┼───────────────────┐      │
│              │    Shared Memory      │                   │      │
│              │  /zed_status          │                   │      │
│              │  /zed_frame           │                   │      │
│              │  /zed_command         │                   │      │
│              └───────────────────────┼───────────────────┘      │
└──────────────────────────────────────┼──────────────────────────┘
                                       │
             ┌─────────────────────────┼─────────────────────────┐
             │                         │                         │
             ▼                         ▼                         ▼
    ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
    │ Python Interface│    │ Apple Vision Pro│    │   Meta Quest    │
    │ (TCP + SHM)     │    │ (UDP Stream)    │    │ (UDP Stream)    │
    └─────────────────┘    └─────────────────┘    └─────────────────┘
```

---

## Installation

### Docker Environment (Recommended)

```bash
# 0. (option) if docker is default
sudo apt-get install -y nvidia-docker2

# 1. Pull ZED SDK Docker image
docker pull stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04

# 2. Clone repository
git clone https://github.com/kodaekwan/MetaQuest_HandTracking.git
cd MetaQuest_HandTracking/StereoStream/ZED_StereoStream

# 3. Run Docker container
xhost +local:root

docker run --gpus all \
    -it \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /dev/shm:/dev/shm \
    -v $(pwd):/app \
    --network=host \
    stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04

# 4. Build inside container
cd /app
chmod +x build_zed_manager.sh
./build_zed_manager.sh

# If you see CMake cache or mismatch errors, clean stale CMake files first:
# (helps when switching between host/container builds)
rm -f CMakeCache.txt
rm -rf CMakeFiles/ build/
cmake -S . -B build
cmake --build build -j4
```

### (Option)Using Dockerfile

```bash
# Build custom image
docker build -t zed-manager:latest .

# Run container
docker run --gpus all \
    -it \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /dev/shm:/dev/shm \
    -v $(pwd):/app \
    --network=host \
    zed-manager:latest
```

### Python Dependencies (Host)

```bash
pip install posix_ipc numpy opencv-python
```

---

## Configuration

### zed_config.json

```json
{
    "name": "zed_camera",
    "width": 1280,
    "height": 720,
    "fps": 30,
    "depth_mode": 0,
    
    "stream_target_ip": "192.168.0.140",
    "stream_port": 9003,
    "stream_width": 640,
    "stream_height": 480,
    "stream_quality": 50,
    
    "stereo_params_port": 9004,
    "stereo_focus": 0.5,
    "stereo_quad": 1.8,
    "stereo_zoom": 1.0,
    "stereo_add_focus": false,
    
    "output_folder": "./recordings",
    "enable_preview": false,
    "control_port": 0
}
```

### Configuration Options

| Option | Description |
|--------|-------------|
| `name` | Camera display name |
| `width`, `height` | Camera resolution |
| `fps` | Frame rate |
| `depth_mode` | 0=NONE, 1=PERFORMANCE, etc. |
| `stream_target_ip` | XR device IP (empty = no auto-start) |
| `stream_port` | UDP streaming port |
| `stream_width/height` | Stream resolution |
| `stream_quality` | JPEG quality (1-100) |
| `stereo_params_port` | TCP port for stereo params (default: 9004) |
| `stereo_focus` | Stereo focus value (0.0-1.0) |
| `stereo_quad` | Stereo quad value |
| `stereo_zoom` | Stereo zoom value |
| `stereo_add_focus` | Additional focus mode (true/false) |
| `control_port` | TCP control port (0 = auto) |

---

## Usage

### Start Camera Manager (C++)

```bash
# Inside Docker container
./zed_camera_manager

# With options
./zed_camera_manager --config zed_config.json
./zed_camera_manager --preview
./zed_camera_manager --port 12345
./zed_camera_manager --stream 192.168.0.140
```

### Command Line Options

| Option | Description |
|--------|-------------|
| `--config <file>` | Configuration file |
| `--port <port>` | TCP control port (0=auto) |
| `--preview` | Enable preview window |
| `--stream <ip>` | Auto-start streaming to IP |
| `--help` | Show help |

### Python Interface CLI

```bash
#[in docker] View mode with shared memory (basic)
python zed_interface.py --action view

#[out docker] View mode with shared memory(basic)
sudo python zed_interface.py --action view

# View mode with TCP controller for streaming control
python zed_interface.py --action view --port <tcp_port> --ip <xr_device_ip>

# Get status via shared memory
python zed_interface.py --action status

# Get status via TCP
python zed_interface.py --action status --port <tcp_port>
```

| Option | Description |
|--------|-------------|
| `--action` | `view`, `status`, `record`, `stream` |
| `--port` | TCP control port (enables streaming toggle) |
| `--host` | Camera manager host (default: localhost) |
| `--ip` | XR device IP for streaming |

### Python Interface

#### TCP Control

```python
from zed_interface import ZedController

# Connect to camera manager
controller = ZedController("localhost", 12345)

# Start streaming to Vision Pro
controller.start_stream(
    ip="192.168.0.140",
    port=9003,
    quality=50,
    width=640,
    height=480
)

# Start recording
result = controller.start_record(
    path="./recordings",
    filename="my_video"
)
print(f"Recording: {result['filepath']}")

# Get status
status = controller.get_status()
print(f"Streaming: {status['streaming']}")
print(f"Recording: {status['recording']}")

# Set stereo parameters for Vision Pro
controller.set_stereo_params(
    target_ip="192.168.0.140",
    focus=0.5,
    quad=1.8,
    zoom=1.0
)

# Stop
controller.stop_record()
controller.stop_stream()
controller.quit()
```

#### Shared Memory (Real-time Frames)

```python
from zed_interface import ZedInterface

# Connect to shared memory
interface = ZedInterface()
interface.connect()

while True:
    # Get frame with timestamps
    frame_data = interface.get_frame(wait_new=True)
    
    if frame_data.valid:
        # Access stereo image (side-by-side)
        stereo_image = frame_data.frame  # numpy array (H, W*2, 3)
        
        # Get timestamps
        camera_us = frame_data.camera_timestamp_us
        system_us = frame_data.system_timestamp_us
        
        # Get interpolated robot time
        robot_us = frame_data.get_interpolated_robot_time_us()
        robot_cam_us = frame_data.get_interpolated_robot_time_cam_us()
        
        # Time since last QR sync
        delta_us = frame_data.get_sync_delta_us()
        
        print(f"Frame {frame_data.frame_counter}, Robot time: {robot_us} us")

interface.disconnect()
```

#### Combined Interface

```python
from zed_interface import ZedCameraInterface

# Combined TCP + Shared Memory
zed = ZedCameraInterface()
zed.connect(host="localhost", port=12345, use_shm=True)

# TCP commands
zed.start_stream("192.168.0.140")
zed.start_record()

# Real-time frame access
while running:
    frame = zed.get_frame(wait_new=True)
    if frame.valid:
        robot_time = frame.get_interpolated_robot_time_us()
        # Process frame...

zed.disconnect()
```

### Keyboard Controls (Python interface test)

| Key | Description |
|-----|-------------|
| `q` | Quit |
| `p` | Toggle preview |
| `r` | Toggle recording |
| `s` | Toggle XR streaming (requires `--port`) |
| `c` | Cycle color mode (BGR/RGB) |

**Note:** Stereo parameters are automatically sent to XR device on startup based on config values.

---

## TCP Commands (JSON)

### Start Streaming
```json
{
    "action": "start_stream",
    "ip": "192.168.0.140",
    "port": 9003,
    "quality": 50,
    "width": 640,
    "height": 480
}
```

### Stop Streaming
```json
{"action": "stop_stream"}
```

### Start Recording
```json
{
    "action": "start_record",
    "path": "./recordings",
    "filename": "video"
}
```

### Stop Recording
```json
{"action": "stop_record"}
```

### Get Status
```json
{"action": "get_status"}
```

**Response:**
```json
{
    "status": "ok",
    "streaming": "true",
    "recording": "true",
    "recording_file": "./recordings/video.mp4",
    "camera_serial": "12345678",
    "control_port": "12345"
}
```

### Set Stereo Parameters (for XR device)
```json
{
    "action": "set_stereo_params",
    "target_ip": "192.168.0.140",
    "target_port": 9004,
    "focus": 0.5,
    "quad": 1.8,
    "zoom": 1.0
}
```

### Shutdown
```json
{"action": "quit"}
```

---

## QR Time Synchronization

Display a QR code containing JSON with PC and robot timestamps. Example payloads accepted by
the QR generator used in this project typically look like:

```json
{
        "PC": "2026-02-14 17:55:25.577",
        "Robot": "2026-02-14 17:55:25.623945+09:00"
}
```

Key behaviors implemented in `zed_camera_manager.cpp`:
- QR detection using ZBar with timer-based scanning (default ~700 ms; faster during initial
    recording/startup to reduce first-frame latency).
- When a QR is detected the manager records: parsed `PC` time, parsed `Robot` time, and the
    ZED camera hardware timestamp at the instant of detection.
- `parseTimestampToUs()` now attempts multiple timezone interpretations (local, UTC, KST) and
    chooses the interpretation closest to the running system time to avoid large offsets when the
    QR-origin host uses a different timezone.
- A microsecond-precision diagnostic is printed in the logs after each QR parse under the
    "[QR] Sync Detail:" output. Look for either:

    - `✅ QR has μs precision`  (parsed PC timestamp includes microseconds)
    - `⚠️ QR seems ms-precision only` (no μs part detected; interpolation may be coarser)

- Interpolation formulas used for per-frame robot time are unchanged conceptually but use
    μs units internally:

```
# System time-based interpolation
interpolated_robot_us = last_qr_robot + (system_timestamp_us - last_qr_pc_timestamp_us)

# Camera time-based interpolation (more precise)
interpolated_robot_cam_us = last_qr_robot + (camera_timestamp_us - last_qr_cam_timestamp_us)
```

Notes & recommendations:
- If you keep seeing large PC↔CAM offsets after these parser heuristics, prefer encoding an
    unambiguous epoch microsecond field (e.g. `"pc_epoch_us": 1676394925577000`) in the QR
    payload — this removes timezone/format ambiguity.
- Check runtime logs for the full `[QR] Sync Detail:` block to verify which timezone
    interpretation was chosen and whether μs precision was detected.

---

## Metadata CSV Format

| Column | Description |
|--------|-------------|
| `frame_number` | Frame sequence number |
| `camera_timestamp_us` | ZED hardware timestamp (μs) |
| `system_timestamp_us` | PC system timestamp (μs) |
| `global_time` | Human-readable time |
| `qr_detected` | QR detected this frame |
| `qr_pc_time` | PC time from QR |
| `qr_robot_time` | Robot time from QR |
| `last_qr_pc_timestamp_us` | Last QR PC time (μs) |
| `last_qr_robot_timestamp_us` | Last QR robot time (μs) |
| `last_qr_cam_timestamp_us` | Camera HW timestamp at QR detection |
| `interpolated_robot_us` | System clock based interpolation |
| `interpolated_robot_cam_us` | Camera clock based interpolation |

---

## File Structure

```
zed_manager_ws/
├── zed_camera_manager.cpp      # Main C++ application
├── zed_shared_memory.h         # Shared memory structures
├── zed_interface.py            # Python interface
├── zed_config.json             # Configuration file
├── CMakeLists.txt              # CMake build configuration
├── build_zed_manager.sh        # Build script
├── Dockerfile                  # Docker build file
├── README.md                   # This file
├── LICENSE                     # MIT License
├── .gitignore                  # Git ignore rules
└── recordings/                 # Default output folder
    ├── zed_camera_*.mp4
    └── zed_camera_*_metadata.csv
```

---

## Docker Image Management

### Save Container as Image

```bash
# Find container ID
docker ps

# Save container
docker commit <CONTAINER_ID> zed-manager:latest
```

### Deployment: Run the Saved Image
This is the standard command to launch your saved zed-manager:latest image with full hardware (GPU/USB) and GUI support.
```bash
# Allow local connections to the X server for GUI tools (ZED Explorer, etc.)
xhost +local:root

# Run the saved image with necessary hardware passthrough
docker run --gpus all \
    -it --rm \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /dev/shm:/dev/shm \
    -v $(pwd):/app \
    --network=host \
    zed-manager:latest
```

### Export/Import Image
Move your configured environment to another workstation without re-building.
```bash
# Export to file
docker save -o zed-manager.tar zed-manager:latest

# Import from file
docker load -i zed-manager.tar
```

---

## Troubleshooting

### Camera Not Detected

```bash
# Check USB connection
lsusb | grep -i stereolabs

# Inside Docker, check ZED
ls /dev/video*

# USB permissions
chmod 666 /dev/bus/usb/*/*
```

### Shared Memory Error

```bash
# Clean up shared memory
rm /dev/shm/zed_*
```

### X11 Display Error

```bash
# On host
xhost +local:root

# Check DISPLAY variable
echo $DISPLAY
```

### Build Errors

```bash
# Check dependencies
pkg-config --exists zed && echo "ZED OK"
pkg-config --exists opencv4 && echo "OpenCV OK"
pkg-config --exists zbar && echo "ZBar OK"
```

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Related Projects

- [Intel RealSense Manager](../realsense_manager_ws) - Similar manager for RealSense cameras
- [ZED SDK](https://www.stereolabs.com/docs/)
- [MetaQuest_HandTracking](https://github.com/kodaekwan/MetaQuest_HandTracking)

---

## Contact

For questions or issues:
- **Daekwan Ko** - Ph.D Student
- **Lab**: Interactive Robotics Lab, Dongguk University
