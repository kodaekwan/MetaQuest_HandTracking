# 🖐️ Meta Quest Hand Tracking Viewer (UDP Receiver for Python)

This repository provides a **Python-based 3D visualizer** that receives real-time hand-tracking data streamed from **Unity3D with Meta Quest**, using **UDP protocol**.

- ✅ Wrist pose (world coordinates)
- ✅ 25 bone poses (relative to wrist, position + rotation)
- ✅ Headset pose (position + rotation)
- ✅ Coordinate system transform (Unity → Robot/World)
- ✅ Real-time 3D rendering with bone connections and axis directions

---

## 📸 Demo

![screenshot](docs/output.gif)  


---

## 🔗 Project Overview

### Unity Side (Transmitter)
- Meta Quest hand-tracking (OVRHand)
- Each hand and headset sends:
  - Wrist pose: position (3 floats) + rotation (quaternion, 4 floats)
  - 25 bones: relative position (3 floats) + rotation (quaternion, 4 floats) each
  - Headset pose: position (3 floats) + rotation (quaternion, 4 floats)
  - Total = `2 * (3+4 + 25*(3+4)) + 7` = `2 * 91 + 7` = 189 floats = 728 bytes × 2 + 28 bytes + header/footer 

### Python Side (Receiver)
- Receives 1500-byte packet:
  - 4-byte header `"HND0"`
  - 8-byte timestamp (`double`)
  - 728 bytes for each hand (182 floats each)
  - 28 bytes for headset
  - 4-byte footer `"HND1"`
- Parses and visualizes each hand's absolute pose in real-time.

---

## 🛠 Requirements

- Python 3.8+
- PyQt5
- PyQtGraph
- NumPy
- SciPy

Install dependencies:
```bash
pip install PyQt5 pyqtgraph numpy scipy
```

---

## 🚀 How to Run

0. **Download [[Sidequest]](https://sidequestvr.com/app/43618/metaquest_handtracking)**(comming soon)
    *This app is available upon request. [Please contact us directly](http://irobot.dgu.edu/).

1. Use SideQuest to install this app on your Meta Quest device.

2. **Run the MetaAPP**

3. Then run this Python visualizer:
```bash
python hand_tracking_udp_receiver.py
```

If you're running on a remote X server or WSL2:
```bash
export DISPLAY=192.168.0.X:0.0
export LIBGL_ALWAYS_INDIRECT=1
python hand_tracking_udp_receiver.py
```

---

## 🧠 Coordinate Transformation

> ✅ Unity uses a **left-handed** coordinate system.  
> ✅ Most robotics systems use a **right-handed** coordinate system.

This code transforms Unity’s **left-handed coordinate system** to a typical robot or world **right-handed coordinate system**:

```python
RM_U2R = np.array([
    [0, 0, 1],    # Unity's z-axis → Robot's x-axis
    [-1, 0, 0],   # Unity's x-axis → Robot's -y-axis
    [0, 1, 0]     # Unity's y-axis → Robot's z-axis
])
```

### Transformations

- **Position transformation**:  
  All bone positions are transformed using:

  ```python
  transformed_pos = RM_U2R @ pos
  ```

- **Rotation transformation**:  
  All rotation matrices are transformed using:

  ```python
  transformed_rot = RM_U2R @ R.as_matrix() @ RM_U2R.T
  ```

  ### Coordinate System Reference

  | System   | +X       | +Y     | +Z       |
  |----------|----------|--------|----------|
  | Unity    | Right    | Up     | Forward  |
  | Robot    | Forward  | Left   | Up       |


---

## 📊 Features

- Real-time 3D bone visualizer with PyQtGraph
- Frame rate + latency monitoring
- Palm and finger connection rendering
- Local axis (XYZ) drawing per joint
- Dual-hand support (left/right)
- Headset support

---

## 📁 Folder Structure

```
.
├── hand_tracking_udp_receiver.py   # Main Python visualizer
├── docs/
│   └── screenshot.png              # Example rendering output
└── README.md
```

---

## 🔒 License

MIT License (feel free to adapt for research or commercial use)

---

## 🙏 Credits

Created by [KO DAEKWAN]  
Meta Quest SDK (OVRHand), Unity3D, PyQtGraph

---

## 🦴 Bone Connection Topology

The hand bone connection structure used for drawing finger joints is based on the XRHand (Meta Quest) indexing:

```python
bone_connection = [
    (0, 1), (1, 2), (2, 3), (3, 4), (4, 5),           # Thumb
    (1, 6), (6, 7), (7, 8), (8, 9), (9, 10),          # Index
    (1, 11), (11, 12), (12, 13), (13, 14), (14, 15),  # Middle
    (1, 16), (16, 17), (17, 18), (18, 19), (19, 20),  # Ring
    (1, 21), (21, 22), (22, 23), (23, 24), (24, 25),  # Little
    (2, 6), (6, 11), (11, 16), (16, 21)               # palm transverse
]
```

This defines:
- Linear finger joint connectivity
- Palm cross-line connections
- Suitable for rendering hand skeletons in 3D