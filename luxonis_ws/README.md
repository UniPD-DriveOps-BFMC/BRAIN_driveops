# 🚗 UniPD-DriveOps — BRAIN Workspace

> Autonomous driving software stack for the **Bosch Future Mobility Challenge (BFMC)** by Team UniPD-DriveOps, University of Padova.

---

## 📦 Repository Structure

```
luxonis_ws/
└── src/
    └── camera_preprocessing/
        ├── camera_preprocessing/
        │   ├── luxonis_preprocessing.py   # Main lane following node
        │   ├── controller3.py             # Lateral/angular controller
        │   └── lane_keeper_small.onnx     # Trained CNN model
        ├── launch/
        │   └── sim.launch.py              # Simulation launch file
        └── deptai-ros/
        └── package.xml
        └── setup.cfg
        └── setup.py
```

---

## 🧠 Packages

### `camera_preprocessing`

CNN-based lane following pipeline using a Luxonis OAK-D camera (real or simulated).

**Pipeline:**
1. Subscribe to `/oak/rgb/image_raw` (`sensor_msgs/Image`, `rgb8`)
2. Preprocess frame: grayscale → crop → resize → Canny → blur → 32×32 blob
3. Run inference with `lane_keeper_small.onnx` to estimate lateral error `e2` and angular error `e3`
4. Compute steering angle via `Controller` (pure pursuit + PD)
5. Publish speed and steering commands to `/automobile/command`

**Key nodes:**

| Node | File | Description |
|------|------|-------------|
| `lane_inference_node` | `luxonis_preprocessing.py` | Main lane following node |

**Key topics:**

| Topic | Type | Direction |
|-------|------|-----------|
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | Subscribed |
| `/automobile/command` | `std_msgs/String` | Published |

---

## ⚙️ Requirements

| Dependency | Version |
|-----------|---------|
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| Python | 3.10+ |
| OpenCV | `cv2` |
| ONNX Runtime | `onnxruntime` |
| cv_bridge | ROS 2 package |

Install Python dependencies:
```bash
pip install onnxruntime opencv-python numpy
```

---

## 🚀 Installation

```bash
# Clone the repository
git clone https://github.com/UniPD-DriveOps-BFMC/BRAIN_driveops.git
cd BRAIN_driveops/luxonis_ws

# Source ROS 2
source /opt/ros/jazzy/setup.bash
# Build
colcon build --packages-select camera_preprocessing
source install/setup.bash
```

---

## 🖥️ Usage

### Run with Gazebo Simulator

```bash
# Source both workspaces
cd BRAIN_driveops/luxonis_ws
source ~/BFMC/bfmc_ws/install/setup.bash
source install/setup.bash

# Launch simulation + lane following
ros2 launch camera_preprocessing sim.launch.py
```

**Launch arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `desired_speed` | `0.3` | Target speed [m/s] |
| `debug_view` | `true` | Show preprocessed frames with OpenCV |

Example with custom parameters:
```bash
ros2 launch camera_preprocessing sim.launch.py desired_speed:=0.1 debug_view:=true
```

### Run on Real Hardware (OAK-D Camera)

First, connect the Luxonis OAK-D camera and launch the camera driver in a **separate terminal**:

```bash
# Terminal 1 — start the Luxonis camera driver
ros2 launch depthai_ros_driver camera.launch.py
```

Then, in a **second terminal**, launch the lane inference node:

```bash
# Terminal 2 — lane following node
ros2 run camera_preprocessing lane_inference_node \
  --ros-args \
  -p input_topic:=/oak/rgb/image_raw \
  -p command_topic:=/automobile/command \
  -p desired_speed:=0.2
```

---

## 🔧 Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_path` | string | bundled `.onnx` | Path to ONNX model |
| `input_topic` | string | `/oak/rgb/image_raw` | Camera topic |
| `command_topic` | string | `/automobile/command` | Command output topic |
| `desired_speed` | float | `0.3` | Target speed [m/s] |
| `debug_view` | bool | `true` | Enable OpenCV debug windows |
| `faster` | bool | `false` | Single inference (no flip augmentation) |

---

## 🧪 Diagnostics

Check topic connectivity:
```bash
ros2 topic info /oak/rgb/image_raw -v
ros2 topic hz /oak/rgb/image_raw
```

Visualize camera feed:
```bash
ros2 run rqt_image_view rqt_image_view
```

View node graph:
```bash
ros2 run rqt_graph rqt_graph
```

## 👥 Team

**UniPD-DriveOps** — University of Padova 
Bosch Future Mobility Challenge 2026

---

## 📄 License

This project is developed for academic and competition purposes. 
© 2026 UniPD-DriveOps — University of Padova
