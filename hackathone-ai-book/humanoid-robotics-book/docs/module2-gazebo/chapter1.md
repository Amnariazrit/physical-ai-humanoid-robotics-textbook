# Gazebo Basics: Environment Setup and Physics Simulation

Gazebo is a powerful 3D robotics simulator that allows you to accurately test algorithms, design robots, and perform complex scenarios in a virtual environment. It's widely used in the robotics community and integrates seamlessly with ROS 2, making it an indispensable tool for developing and validating robotic systems.

This chapter will introduce you to the fundamentals of Gazebo, covering its basic components, environment setup, and how it handles physics simulation. We will focus on Ignition Gazebo (formerly Gazebo Fortress), the modern successor to the classic Gazebo simulator, which offers improved performance and a more modular architecture.

## What is Gazebo?

Gazebo is not just a rendering tool; it's a full-fledged physics engine. It simulates:

*   **Rigid Body Dynamics**: How objects move and interact under physical forces (gravity, collisions, friction).
*   **Sensor Noise**: Realistic sensor data from various sensors (cameras, LiDAR, IMU, sonar, etc.).
*   **Environmental Interactions**: Lighting, wind, water, and other environmental factors.
*   **Robot Dynamics**: Simulating joint limits, motor dynamics, and control interfaces.

By using Gazebo, you can develop and debug complex robot behaviors in a safe and repeatable virtual space, significantly reducing development time and costs compared to working solely with physical hardware.

## Key Components of Gazebo

Gazebo is composed of several key components that work together to create a realistic simulation:

1.  **World**: The "world" is the environment where the simulation takes place. It defines the terrain, objects (buildings, furniture, obstacles), light sources, and physics properties (like gravity). Worlds are typically defined in `.world` files (XML format based on SDF).
2.  **Models**: Models represent individual objects within the world, including robots, static objects (e.g., tables, walls), and dynamic objects (e.g., boxes to be manipulated). Models are usually defined using URDF (Unified Robot Description Format) or SDF (Simulation Description Format) files.
3.  **Physics Engine**: Gazebo uses various physics engines (e.g., ODE, Bullet, DART, Simbody) to calculate rigid body dynamics, collisions, and friction. You can choose the engine best suited for your simulation needs.
4.  **Sensors**: Gazebo simulates a wide range of sensors, including cameras (monocular, stereo, depth), LiDAR (2D and 3D), IMUs (Inertial Measurement Units), contact sensors, and more. These simulated sensors provide data that closely mimics real-world sensor output, often including configurable noise models.
5.  **Plugins**: Gazebo's functionality can be extended through plugins. These are shared libraries that can be loaded into the simulator to add custom robot behaviors, sensor models, or world features. Many ROS 2 integrations with Gazebo rely on plugins.
6.  **Graphical User Interface (GUI)**: Gazebo comes with a powerful GUI that allows you to visualize the simulation in 3D, interact with models, inspect their properties, and control simulation playback (play, pause, step).

## Environment Setup for Gazebo with ROS 2 Humble

This textbook assumes you have ROS 2 Humble installed on Ubuntu 22.04. Gazebo is often installed alongside ROS 2, but for Ignition Gazebo, a separate installation is typically needed.

### 1. Install Ignition Gazebo Fortress

If you haven't already, install Ignition Gazebo Fortress:

```bash
sudo apt-get update
sudo apt-get install ignition-fortress
```

### 2. Install ROS 2 - Gazebo Bridge

To enable communication between ROS 2 nodes and Gazebo, you need the `ros_gz_bridge` package. This package provides nodes that translate messages between ROS 2 and Ignition Transport (Gazebo's internal communication system).

```bash
sudo apt-get install ros-humble-ros-gz
```

### 3. Verify Installation

You can test your Gazebo installation by launching an empty world:

```bash
ign gazebo empty.sdf
```

This command should open the Gazebo GUI with an empty 3D world. You can add simple models (like a sphere or box) from the "Insert" tab on the left panel to interact with them.

To verify the ROS 2 bridge, you can run a simple example that publishes a clock message:

```bash
# In one terminal, run the Gazebo bridge clock publisher
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock

# In another terminal, check if the ROS 2 clock topic is available
ros2 topic echo /clock
```

You should see ROS 2 clock messages being published, confirming the bridge is functional.

## Physics Simulation Basics

Gazebo's physics engine is responsible for accurately simulating how objects behave in the physical world. Key aspects include:

*   **Gravity**: By default, Gazebo simulates Earth's gravity (-9.8 m/s² in the Z-direction). You can modify this in your `.world` file.
*   **Collisions**: When two simulated objects come into contact, the physics engine calculates collision forces and prevents them from passing through each other. Proper collision geometry is crucial for realistic interactions.
*   **Joints**: Robot joints are simulated with their specified types (revolute, prismatic, fixed) and limits. This allows for accurate kinematic and dynamic behavior of articulated robots.

Understanding these basics is the first step toward creating effective and realistic robotics simulations. The next chapters will dive deeper into defining custom models and environments.
