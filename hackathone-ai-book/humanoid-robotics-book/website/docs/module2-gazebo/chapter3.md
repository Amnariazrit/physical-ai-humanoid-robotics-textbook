# URDF and SDF Formats: Robot and Environment Descriptions

To effectively simulate robots in Gazebo, you need a way to describe their physical properties, kinematic and dynamic structures, and sensor configurations. Similarly, the simulation environment itself needs to be defined, including static objects, terrains, and lighting. This is where the Unified Robot Description Format (URDF) and Simulation Description Format (SDF) come into play.

While we briefly touched upon URDF in Module 1, this chapter will compare URDF and SDF, explaining their respective strengths and how they are used together to create comprehensive robot and world descriptions for Gazebo.

## Understanding URDF Structure

As discussed, URDF is an XML-based format primarily designed for describing a robot's kinematics (links and joints), visual appearance, and collision properties for visualization and some basic simulation tasks. Its strengths lie in its simplicity for representing a single robot.

### Key Characteristics of URDF:

*   **Single Robot Focus**: Designed to describe one robot in isolation.
*   **Kinematic Tree**: Represents a robot as a tree structure, starting from a base link, with joints connecting parent-child links. This means a single root link and no closed loops.
*   **Visualization and Collision**: Excellent for defining `visual` and `collision` properties for tools like RViz and basic physics.
*   **Extensibility with Xacro**: Often used with Xacro to improve modularity and readability.

## Simulation Description Format (SDF)

SDF is a more comprehensive XML format specifically designed for describing everything in a Gazebo simulation: robots, environments, sensors, and even light sources. It's a superset of URDF in terms of capabilities, able to represent more complex scenarios.

### Key Characteristics of SDF:

*   **World and Robot Description**: Can describe entire worlds, including multiple robots, static objects, and environmental features.
*   **Graph Structure**: Supports arbitrary graph structures, allowing for closed kinematic loops (e.g., parallel manipulators) that URDF cannot directly represent.
*   **Plugins**: Directly supports Gazebo plugins, enabling custom behaviors for models and worlds.
*   **Sensors**: Rich description for various sensor types, including their physical properties and noise models.
*   **Physics Properties**: Fine-grained control over physics parameters for individual links and joints, as well as global world physics settings.

## Comparing URDF and SDF

| Feature               | URDF                                     | SDF                                                                |
| :-------------------- | :--------------------------------------- | :----------------------------------------------------------------- |
| **Primary Use**       | Single robot description                 | Complete world description (robots, objects, environment)            |
| **Structure**         | Kinematic tree (no closed loops)         | General graph (supports closed loops)                              |
| **Physics**           | Basic (visuals, collisions, inertia)     | Comprehensive (full dynamics, friction, contacts, engines)         |
| **Sensors**           | Limited; often added via ROS wrappers    | Rich, native sensor definitions with noise models                  |
| **Plugins**           | No native support; ROS plugins via `ros_control` | Native Gazebo plugin support for models and worlds                 |
| **Lights**            | No                                       | Yes, native light source definitions                               |
| **Typical ROS Use**   | `robot_state_publisher` for visualization | `ros_gz_bridge` for simulation data & control                     |

## Using URDF and SDF Together

Despite their differences, URDF and SDF are often used in conjunction within ROS 2 and Gazebo.

*   **URDF for Robot Model**: It's common to define the core robot model in URDF (often via Xacro) due to its simplicity and existing tooling.
*   **Conversion to SDF**: Gazebo typically converts URDF files to its internal SDF representation when loading models. You can also explicitly convert an URDF to SDF using tools like `gz sdf --print-from-urdf <urdf_file>`.
*   **SDF for World**: The overall simulation environment, including the ground plane, walls, and any static objects, is best defined directly in an SDF `.world` file.
*   **Embedding Robots**: Robots defined in URDF or separate SDF files can then be included as `<model>` elements within a larger SDF `.world` file.

## Python Example: Basic Gazebo SDF Environment

Let's create a simple SDF `.world` file that defines a flat ground plane and a simple box in the environment. This forms the basis for any simulation where you want your robot to interact with static objects.

Create the file `simple_box_world.sdf` in `static/code-examples/module2/ch3_sdf/`.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="my_box">
      <pose>0 0 0.5 0 0 0</pose> <!-- x, y, z, roll, pitch, yaw -->
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.0 0.0 1.0 1</ambient>
            <diffuse>0.0 0.0 1.0 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>0 0 0 1</emissive>
          </material>
        </visual>
      </link>
      <static>true</static> <!-- Make the box static -->
    </model>

    <!-- You can add more models or modify the environment here -->

  </world>
</sdf>
```

To run this world in Gazebo:

```bash
ign gazebo -v 4 -r static/code-examples/module2/ch3_sdf/simple_box_world.sdf
```

This will launch Gazebo with a ground plane, a directional light source (sun), and a blue 1x1x1 meter static box positioned at (0, 0, 0.5) meters.

## Conclusion

URDF and SDF are complementary formats essential for defining robots and their environments in Gazebo. While URDF excels at describing a single robot's kinematic and visual properties, SDF provides a more extensive framework for entire simulation worlds, including physics, sensors, and multiple models. Understanding how to use both, and how Gazebo interprets them, is key to creating rich and functional digital twins for your humanoid robots.
