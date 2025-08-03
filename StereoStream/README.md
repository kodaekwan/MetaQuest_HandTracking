# StereoStream

> **Note:** This feature will be supported in the app update scheduled for next year.

---

## 📸 Demo

![screenshot](../docs/output2.gif)  


---

## 📖 Overview

`StereoStream` is a Python utility that captures stereo images from a RealSense D405 camera, encodes them as JPEGs, splits the data into UDP packets, and streams them to external clients such as Meta Quest devices.

* **UdpImageSender** class in StereoStreamer: Enables reliable UDP streaming with configurable image dimensions, resolution, JPEG quality, and maximum payload size.
* **example.py**: Demonstrates how to capture frames from a RealSense camera and send them using `UdpImageSender`.

## 🚀 Key Features

* JPEG encoding and automatic splitting into 60 KB UDP packets
* Fixed image resolution (`width` × `height`) specified at class initialization
* Flexible configuration of target IP/port, JPEG quality, and UDP payload size
* Simple API: `open()`, `connect()`, `send_image()`, and `close()` methods

## 📦 Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/kodaekwan/MetaQuest_HandTracking.git
   cd MetaQuest_HandTracking
   ```

2. Create and activate a virtual environment:

   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. Install dependencies:

   ```bash
   pip install opencv-python numpy pyrealsense2
   
   ```
   option install(improve encode)
   ```bash
   sudo apt-get install libturbojpeg
   pip install pyTurboJPEG
   ```


## ⚙️ Environment Variables

If you need X11 forwarding or indirect OpenGL rendering on Linux, set:

```bash
export DISPLAY=<METAQUEST_IP>:1.0
export LIBGL_ALWAYS_INDIRECT=1
```

## 📁 Project Structure

```
StereoStreame/
├── StereoStreamer.py             # Definition of UdpImageSender class
├── camera_datacollection.py      # Definition of RealsenseCamera class
├── example.py                    # Example script: RealSense → UDP streaming
└── README.md                     # Project documentation in Markdown
```

## 📝 Usage

1. Place `StereoStreamer.py` and `example.py` in the same directory.
2. Edit `example.py` to match your environment (IP, port, resolution, etc.).
3. Run the example:

   ```bash
   python example.py
   ```

### example.py

```python
from StereoStreamer import UdpImageSender
from camera_datacollection import RealsenseCamera
import cv2
import time
import numpy as np

# Initialize RealSense camera
cam = RealsenseCamera(
    name_keyword="D405", height=480, width=640,
    fps=30, use_color=True, use_depth=False,
    use_streo=True, reset_on_start=True
)
cam.start_reader()

# Create UDP image sender
sender = UdpImageSender(
    ip='192.168.0.133', port=9003,
    width=1280, height=480,
    max_payload=60*1024, jpeg_quality=50
)
sender.open()
sender.connect()

# Prepare a buffer for combined stereo image
black_img = np.zeros((480, 1280, 3), dtype=np.uint8)

try:
    while True:
        if cam.frame_queue:
            _, _, left, right = cam.frame_queue.popleft()
            # Merge left and right images side by side
            black_img[:, :640] = cv2.cvtColor(left, cv2.COLOR_GRAY2BGR)
            black_img[:, 640:] = cv2.cvtColor(right, cv2.COLOR_GRAY2BGR)
            # Send merged image over UDP
            sender.send_image(black_img)
        time.sleep(1/30)
finally:
    sender.close()
```

## 📄 License

This project is distributed under the MIT License.
