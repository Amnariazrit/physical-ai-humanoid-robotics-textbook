# URDF for Humanoids: Robot Description and Simulation Basics

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe the physical characteristics of a robot. It allows you to define the robot's kinematics (links and joints), visual properties (geometry and colors), and collision properties (shapes for physics simulation). For humanoid robots, URDF is crucial for accurately representing their complex structure, enabling simulation, visualization, and motion planning.

This chapter will introduce the fundamentals of URDF, focusing on its application to humanoid robots. We will cover how to define links, joints, and transmissions, and how to use these descriptions for visualization in tools like RViz and for basic simulation in Gazebo.

## Understanding URDF Structure

A URDF file is an XML document with a `<robot>` root element. Inside, it defines two primary elements:

1.  **`<link>`**: Represents a rigid body segment of the robot (e.g., torso, upper arm, thigh). Links have physical properties like mass, inertia, and visual/collision geometry.
2.  **`<joint>`**: Describes the connection between two links. Joints define the degrees of freedom (DOF) and the kinematic relationship between parent and child links.

### Key Elements within URDF

*   **`<link>` Elements**:
    *   **`<visual>`**: Defines the graphical representation of the link (e.g., mesh file, primitive shape like a box or cylinder) and its color. This is what you see in RViz.
    *   **`<collision>`**: Defines the geometry used for collision detection in physics simulations. It's often a simplified version of the visual geometry.
    *   **`<inertial>`**: Specifies the link's mass, center of mass (origin), and inertia matrix. Essential for accurate physics simulation.
*   **`<joint>` Elements**:
    *   **`name`**: Unique identifier for the joint.
    *   **`type`**: Defines the type of joint (e.g., `revolute`, `prismatic`, `fixed`, `continuous`). Humanoids typically use `revolute` for rotating joints (like elbows, knees) and `fixed` for rigid connections.
    *   **`<parent>` and `<child>`**: Specify the `link` elements that the joint connects.
    *   **`<origin>`**: Defines the transform from the parent link's origin to the joint's origin.
    *   **`<axis>`**: For revolute/prismatic joints, specifies the axis of rotation/translation.
    *   **`<limit>`**: For revolute/prismatic joints, defines the upper and lower limits of the joint's movement, and its velocity/effort limits.
    *   **`<dynamics>`**: Defines friction and damping properties.
    *   **`<mimic>`**: Allows one joint to mimic the motion of another joint.
*   **`<transmission>` (Xacro/URDF Extension)**:
    *   While not strictly part of core URDF, `transmission` elements are often used with URDFs, especially for hardware interfacing. They describe the relationship between actuators and joints, crucial for `ros2_control`.

## Xacro: Simplifying URDF

Writing complex URDF files manually can be tedious and prone to errors, especially for humanoids with many links and joints. Xacro (XML Macros) is an XML macro language that allows you to use macros, properties, and arithmetic operations within your URDF, making it more modular, readable, and easier to maintain.

You convert a `.xacro` file to a `.urdf` file using the `xacro` command-line tool.

## Generic Humanoid URDF Model

Below is a simplified example of a generic humanoid URDF structure. This example focuses on the basic kinematic chain for a torso, head, and single arm, demonstrating links and revolute joints. A full humanoid would extend this significantly for legs, a second arm, hands, and feet.

### Example: `generic_humanoid.urdf.xacro`

To keep the example concise and modular, we will use Xacro.

```xml
<?xml version="1.0"?>
<robot name="generic_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Define common properties/macros -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="body_mass" value="1.0" />
  <xacro:property name="body_len" value="0.2" />
  <xacro:property name="body_rad" value="0.05" />

  <!-- Macro for a generic cylinder link with inertia -->
  <xacro:macro name="cylinder_link" params="name mass length radius">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
        <material name="blue">
          <color rgba="0 0 0.8 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="${(1/12)*mass*(3*radius*radius + length*length)}" ixy="0.0" ixz="0.0"
                 iyy="${(1/12)*mass*(3*radius*radius + length*length)}" iyz="0.0"
                 izz="${(1/2)*mass*radius*radius}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base Link: Torso -->
  <xacro:cylinder_link name="torso_link" mass="${body_mass}" length="${body_len}" radius="${body_rad}"/>

  <!-- Head Joint and Link -->
  <joint name="head_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 ${body_len/2 + body_len/4}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="10.0" velocity="1.0"/>
  </joint>
  <xacro:cylinder_link name="head_link" mass="${body_mass/4}" length="${body_len/2}" radius="${body_rad/2}"/>

  <!-- Right Shoulder Joint and Link -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="right_upper_arm_link"/>
    <origin xyz="0 ${-(body_rad + body_len/4)} 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="10.0" velocity="1.0"/>
  </joint>
  <xacro:cylinder_link name="right_upper_arm_link" mass="${body_mass/4}" length="${body_len}" radius="${body_rad/3}"/>

  <!-- Right Elbow Joint and Link -->
  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm_link"/>
    <child link="right_forearm_link"/>
    <origin xyz="0 0 ${-body_len/2}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${0}" effort="10.0" velocity="1.0"/>
  </joint>
  <xacro:cylinder_link name="right_forearm_link" mass="${body_mass/6}" length="${body_len}" radius="${body_rad/4}"/>

</robot>
