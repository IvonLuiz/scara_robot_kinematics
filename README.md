# SCARA Kinematics Analysis

## 📌 Objective

Use the Denavit–Hartenberg (DH) convention to derive the forward kinematics model for a SCARA manipulator. For this task:

- Assume all links are 1.0 c.u. in length.
- The base is elevated at 0.5 c.u.
- Compute the end-effector pose for 10 different sets of joint values.
- Validate your DH-based results using the [`scara_robot`](https://github.com/aniketmpatil/scara_robot) ROS package.

You are expected to submit a short report or video demonstrating the development process and the obtained results.

---

## 🛠️ Development Setup

### 🐳 Using Docker (Recommended)

To build the Docker image:

```bash
docker build . -t scara_kinematics
```

To run the container with GUI and ROS networking support:

```bash
docker run -it --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PWD/../ros_ws:/home/catkin_ws" \
    --name scara_container \
    scara_kinematics
```
### 🚀 Usage

1. **Launch the Simulation Environment**  
    In Terminal 1, start the SCARA simulation environment:
    ```bash
    roslaunch scara_robot my_env.launch
    ```

2. **Run Forward Kinematics Node**  
    In Terminal 2, compute the forward kinematics from the simulated robot:
    ```bash
    rosrun scara_robot scara_forward
    ```

3. **Run DH-Based Validation Script**  
    In Terminal 3, validate your DH-based results:
    ```bash
    rosrun scara_tests test_fk.py
    ```
    This script will:
    - Publish 10 test cases of joint values.
    - Compute the end-effector pose using your DH-derived transformation.
    - Retrieve the simulated pose from the SCARA robot.
    - Log and optionally plot the differences between both results.

---

### ⚙️ Adjusting for Consistency

The reference robot's base height differs from the 0.5 c.u. specified above. To align the Z-coordinate of both systems:

1. **Modify URDF Base Height**  
    Edit `ros_ws/src/scara_robot/urdf/robot_desc.xacro`:
    ```xml
    <xacro:property name="height0" value="0.3" />
    ```
    Change to:
    ```xml
    <xacro:property name="height0" value="0.5" />
    ```

2. **Update Forward Kinematics in C++**  
    Edit `ros_ws/src/scara_robot/src/forward.cpp`:
    ```cpp
    static const double h0 = 0.3;
    ```
    Change to:
    ```cpp
    static const double h0 = 0.5;
    ```

This ensures your DH model and the robot's simulated kinematics use the same base height.

