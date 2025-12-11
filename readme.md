
# ROS 2 Projectile Simulator

[](https://docs.ros.org/en/humble/index.html)
[](https://opensource.org/licenses/MIT)
[](https://www.python.org/)

> **A lightweight visualization tool for simulating projectile kinematics with air resistance in RViz 2.**

-----

\<div align="center"\>
\<img src="[https://via.placeholder.com/800x400.png?text=Projectile+Simulation+Demo](https://www.google.com/search?q=https://via.placeholder.com/800x400.png%3Ftext%3DProjectile%2BSimulation%2BDemo)" alt="Demo" width="100%"\>
<br>
\<em\>Figure 1: Visualization of 17mm projectiles. Cyan spheres indicate active rounds with fading trails; the light grey dashed line shows the real-time predicted trajectory based on current launcher pose.\</em\>
\</div\>

-----

## 📖 Overview

This package provides a standalone ROS 2 node (`launch_sim_node`) that simulates the flight path of small spherical projectiles (e.g., 17mm robotic competition rounds).

Unlike simple parabolic visualizations, this simulator implements a **physics-based update loop** that accounts for **aerodynamic drag**. It is designed to help developers debug aiming algorithms by providing a visual comparison between a theoretical prediction and a physics-aware flight path.

## ✨ Key Features

  - **Physics Simulation:** Implements Euler integration for gravity and air drag force ($F_d \propto v^2$).
  - **Real-Time Prediction:** Visualizes the predicted impact trajectory (dashed line) relative to the launcher's current TF frame.
  - **Visual Feedback:**
      - **Projectiles:** Rendered as Cyan spheres (`MarkerArray`).
      - **Trails:** Velocity-dependent, time-decaying trails to visualize flight history.
  - **Auto-Expiry:** Visual markers automatically expire to prevent "ghosting" and keep RViz clean.
  - **Configurable:** All physical properties (mass, radius, drag coefficient) are tunable via YAML.

## 🛠️ Installation

### Dependencies

  * ROS 2 (Humble or Jazzy)
  * Python 3
  * Standard ROS libraries (`geometry_msgs`, `visualization_msgs`, `tf2_ros`)

### Build Steps

```bash
# 1. Clone the repository into your workspace src folder
cd ~/ros2_ws/src
git clone <repository_url> launch_sim

# 2. Build the package
cd ~/ros2_ws
colcon build --packages-select launch_sim

# 3. Source the setup script
source install/setup.bash
```

## 🚀 Usage

### 1\. Launch the Simulation

This command starts the simulator node and a static TF publisher (mimicking a launcher mounting point).

```bash
ros2 launch launch_sim 17mm.launch.py
```

### 2\. Configure RViz 2

Open RViz 2 and add the following **Marker** displays:

| Display Name | Topic | Description |
| :--- | :--- | :--- |
| **Balls** | `/visual/balls` | The simulated projectiles (Cyan). |
| **Trails** | `/visual/trails` | The flight path history (Cyan/Transparent). |
| **Prediction** | `/visual/prediction` | The theoretical trajectory line (Grey). |

*Note: Ensure the "Fixed Frame" in RViz is set to `map`.*

## ⚙️ Parameters

Configuration is handled in `config/ball_params.yaml`.

| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `mass` | `0.0032` | kg | Mass of the projectile (Standard 17mm is \~3.2g). |
| `radius` | `0.0085` | m | Radius of the projectile. |
| `initial_speed` | `28.0` | m/s | Muzzle velocity. |
| `drag_coeff` | `0.47` | - | Drag coefficient for a sphere. |
| `air_density` | `1.225` | kg/m³ | Air density at sea level. |
| `shoot_interval` | `0.15` | s | Interval for automatic firing test. |
| `trail_lifetime` | `0.4` | s | Duration trails remain visible. |

## 🧮 Physics Model

The simulation calculates acceleration at each time step using the drag equation:

$$\vec{a} = \vec{g} - \frac{1}{2m} \rho C_d A v \vec{v}$$

Where:

  * $\vec{g}$: Gravity vector $(0, 0, -9.81)$
  * $\rho$: Air density
  * $C_d$: Drag coefficient
  * $A$: Cross-sectional area ($\pi r^2$)
  * $v$: Velocity magnitude

This ensures that light projectiles (like the 3.2g 17mm ball) exhibit realistic deceleration and drop compared to a vacuum trajectory.

## 📄 License

This project is open-source and available under the [MIT License](https://www.google.com/search?q=LICENSE).