# ABB IRB 4600 Automotive Painting Simulation

A ROS 2 and MoveIt2 based simulation of an automotive robotic painting cell using an ABB IRB 4600 industrial robot mounted on a linear rail.

The system generates surface-based painting waypoints from a 3D vehicle model and executes collision-aware robot trajectories to simulate automated painting of the vehicle roof and near-side door.

## Project Overview

The objective of this project is to develop a robotic painting simulation that demonstrates industrial robot kinematics, workspace analysis, inverse kinematics, trajectory planning, and collision checking.

The simulation consists of:

- ABB IRB 4600-40/2.55 six-axis industrial robot.
- A 7.4 m linear rail for robot movement along the vehicle.
- A vehicle mounted on a turntable.
- A Y-axis slide for repositioning the vehicle.
- A spray gun attached to the robot flange.
- A ROS 2 MoveIt 2 motion-planning framework.

The simulated painting tasks include:
1. Near-side roof painting.
2. Far-side roof painting using turntable rotation.
3. Near-side door painting using vehicle repositioning and spray-gun reorientation.

## System Architecture

| Component | Specification |
|---|---|
| Robot | ABB IRB 4600-40/2.55 |
| Robot configuration | 6 revolute joints |
| Linear rail | 7.4 m, approximately ±3.2 m |
| Turntable | Approximately ±180° |
| Vehicle slide | Y-axis linear movement |
| Cell dimensions | 12 × 8 m |
| Spray gun TCP offset | 0.31 m |
| Spray standoff distance | 0.28 m |
| Robot pedestal height | 0.5 m |

## Software and Frameworks

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- RViz2
- Python 3
- C++
- Eigen3
- KDL kinematics
- OMPL RRTConnect planner

## Technical Approach

### 1. Robot Kinematics

A Denavit-Hartenberg based mathematical model was developed for the ABB IRB 4600.

Forward kinematics was used to calculate the end-effector position and orientation from joint configurations.

Inverse kinematics was studied using an analytical solution based on the spherical-wrist configuration. The analytical solution was validated against 300 randomly generated poses.

The MoveIt simulation itself uses KDL numerical inverse kinematics.

### 2. Jacobian Analysis

The geometric Jacobian was derived to relate joint velocities to end-effector linear and angular velocities.

A 6 × 7 Jacobian was considered for the combined linear rail and six-axis robot configuration.

The analytical Jacobian was compared against a numerical Jacobian, with a maximum error of approximately 3.85 × 10⁻¹⁰ in the tested configuration.

These calculations were used for kinematic validation and study.

### 3. Workspace and Reachability Analysis

The robot workspace was evaluated to identify suitable mounting positions and vehicle configurations.

A 0.5 m pedestal was introduced to improve reachability for the tested roof-painting poses.

The simplified shoulder-to-wrist reach estimate was approximately 2.377 m. This is a geometric estimate and should not be confused with the manufacturer's rated 2.55 m reach.

### 4. Surface-Based Waypoint Generation

The vehicle visual mesh was used for ray-casting to identify candidate painting points.

Separate waypoint-generation procedures were developed for:

- Roof surfaces.
- Far-side roof surfaces.
- Near-side door surfaces.

A nominal spray standoff distance of 0.28 m was used.

The generated waypoints are stored as JSON files in the `tools` directory.

### 5. Motion Planning

MoveIt 2 is used for inverse kinematics and collision-aware motion planning.

The implemented sequence for each painting stripe is:

1. Move the linear rail to the stripe position.
2. Move the robot to a hover position.
3. Descend to the first painting waypoint.
4. Execute the stripe using Cartesian path interpolation.
5. Lift the spray gun away from the vehicle.

The main planning parameters include:

| Parameter | Value |
|---|---:|
| Cartesian interpolation step | 0.02 m |
| Minimum accepted Cartesian fraction | 0.99 |
| Maximum adjacent joint change | 0.6 rad |
| Joint speed limit | 0.15 rad/s |
| Hover distance | 0.15 m |
| Spray standoff | 0.28 m |

The rail remains fixed while each individual stripe is executed.

### 6. Collision Checking

MoveIt planning-scene collision checking is enabled for the relevant robot, spray-gun, and vehicle interactions.

A simplified vehicle collision mesh is used to reduce computational complexity.

An intentional spray-gun/vehicle intersection was tested. MoveIt detected the collision and rejected the invalid goal. A subsequent collision-free target was successfully planned and executed.

Collision checking is performed against the simulated geometry and does not constitute physical industrial safety certification.

## Simulation Results

The completed simulated painting workflow produced the following results:

| Painting section | Stripes | Waypoints |
|---|---:|---:|
| Near-side roof | 7 | 40 |
| Far-side roof | 7 | 42 |
| Near-Side door | 4 | 54 |
| **Total** | **18** | **136** |

The far-side roof was accessed by rotating the vehicle approximately 180° on the turntable.

The Near-side door painting sequence used a 1.0 m vehicle slide and a sideways spray-gun orientation.

The painting trail is a visualization of the simulated spray path. It does not represent actual paint deposition, film thickness, or industrial paint quality.

## Repository Structure

```text
abb-irb4600-car-painting-simulation/
│
├── README.md
├── .gitignore
│
└── src/
    ├── painting_cell_description/
    │   ├── urdf/
    │   ├── meshes/
    │   ├── launch/
    │   ├── rviz/
    │   ├── kinematics/
    │   └── tools/
    │       ├── paint_roof.py
    │       ├── paint_far.py
    │       ├── paint_side.py
    │       ├── check_reach.py
    │       └── waypoint JSON files
    │
    ├── painting_cell_moveit_config/
    │   ├── config/
    │   └── launch/
    │
    └── moveit_jacobian_test/
        └── src/
            └── moveit_jacobian.cpp
```

## Installation and Execution

### Prerequisites

The project requires Ubuntu 22.04, ROS 2 Humble, and the corresponding MoveIt 2 packages.

Install the ROS 2 and MoveIt dependencies required by the project before building.

### Build the Workspace

Clone the repository and enter its directory.

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Build:

```bash
colcon build --packages-up-to painting_cell_moveit_config moveit_jacobian_test
```

Source the workspace:

```bash
source install/setup.bash
```

### Launch the MoveIt Simulation

Start the MoveIt demo:

```bash
ros2 launch painting_cell_moveit_config demo.launch.py
```

Wait for the robot model, planning scene, and MoveIt services to become available.

### Execute the Painting Scripts

Open separate terminals, source the ROS 2 and workspace setup files, and execute the required scripts.

Near-side roof:

```bash
python3 src/painting_cell_description/tools/paint_roof.py all
```

Far-side roof:

```bash
python3 src/painting_cell_description/tools/paint_far.py
```

Near-side door painting:

```bash
python3 src/painting_cell_description/tools/paint_side.py
```

The painting scripts require the MoveIt demo and relevant ROS 2 services to be running.

## Important Notes

- The painting scripts use the JSON waypoint files included in the repository.
- The `prepare_car.py` and `simplify_car.py` scripts are mesh-preparation utilities. Review their output paths and settings before executing them.
- Third-party Nissan Altima 2013 vehicle meshes are not included in this public repository due to redistribution considerations.
- To run the complete vehicle simulation, obtain the Nissan Altima 2013 model from [GrabCAD](https://grabcad.com/library/nissan-altima-2013-1), subject to its applicable terms, and use `prepare_car.py` to generate the required visual and collision meshes locally.
- The painting trail is a visualization, not a physical paint simulation.
- The project demonstrates simulated motion planning and collision checking, not validated physical robot operation.

## Acknowledgements

- **ABB Robotics / ROS-Industrial:** Robot description and mesh resources for the ABB IRB 4600 model. The original ABB-related resources are associated with the [ROS-Industrial ABB repository](https://github.com/ros-industrial/abb).
- **Vehicle 3D Model:** Nissan Altima 2013 model sourced from [GrabCAD](https://grabcad.com/library/nissan-altima-2013-1). Original model creator: Jorge Cruz. The vehicle meshes are not redistributed in this public repository.

All third-party models and resources remain the property of their respective creators and are subject to their original licences and permissions.

## Author

**Kaushik P**

Robotics and Artificial Intelligence Engineer

Portfolio Website: [https://kaushikportfolio.com]
LinkedIn: [www.linkedin.com/in/kaushik08]
GitHub: [https://github.com/kaushikurwa]
Email: [kaushikurwa@gmail.com]
