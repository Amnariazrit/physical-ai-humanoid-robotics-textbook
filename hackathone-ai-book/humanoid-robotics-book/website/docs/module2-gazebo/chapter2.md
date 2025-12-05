# Physics Simulation: Gravity, Collisions, Dynamics

Gazebo's ability to accurately simulate physical interactions is central to its utility in robotics development. This chapter will delve deeper into the core components of physics simulation within Gazebo, covering how to configure gravity, understand and manage collisions, and work with the dynamic properties of models. A strong grasp of these concepts is essential for creating realistic and predictable robot behaviors in your virtual environments.

## Gravity

Gravity is a fundamental force in any physics simulation. By default, Gazebo worlds are configured with Earth's gravity, acting downwards along the Z-axis.

### Modifying Gravity

You can modify the gravity vector within your `.world` file. This is useful for simulating environments with different gravitational forces (e.g., lunar exploration) or for specific testing scenarios where you might want to disable gravity temporarily.

A `gravity` element within the `<physics>` tag in your `.world` file defines the vector:

```xml
<physics name="default_physics" default="0">
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.0</contact_surface_layer>
    </constraints>
  </ode>
  <gravity>0 0 -9.8</gravity> <!-- Default Earth gravity -->
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

To disable gravity, set all components of the vector to zero: `<gravity>0 0 0</gravity>`.

## Collisions

Collision detection and response are critical for preventing objects from interpenetrating and for simulating realistic interactions. In Gazebo, each `<link>` in a model can have one or more `<collision>` elements.

### Defining Collision Geometry

A `<collision>` element specifies the geometric shape used by the physics engine for collision detection. It should generally be a simpler representation than the visual geometry to reduce computational overhead. Common collision shapes include:

*   **`<box>`**: A rectangular prism.
*   **`<cylinder>`**: A cylinder.
*   **`<sphere>`**: A sphere.
*   **`<mesh>`**: A triangular mesh, typically loaded from a `.dae` (Collada) or `.stl` file. Use meshes sparingly for collision, as they are computationally expensive.

Each `<collision>` element also has an `<origin>` (position and orientation relative to the link) and `<geometry>`.

Example within a `<link>`:

```xml
<link name="base_link">
  <collision name="base_collision">
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>    <!-- Coefficient of friction -->
          <mu2>1.0</mu2>   <!-- Secondary coefficient of friction -->
          <slip1>0.0</slip1>
          <slip2>0.0</slip2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.1</restitution_coefficient> <!-- Bounciness -->
        <threshold>0.0</threshold>
      </bounce>
      <contact>
        <collide_bitmask>1</collide_bitmask> <!-- For selective collision -->
      </contact>
    </surface>
  </collision>
  <!-- ... other link elements like visual, inertial ... -->
</link>
```

### Collision Properties (`<surface>`)

Within a `<collision>` element, the optional `<surface>` tag allows you to define material properties that affect contact interactions:

*   **`<friction>`**: Defines coefficients of friction (`mu`, `mu2`) and sliding friction (`slip`).
*   **`<bounce>`**: Defines how elastic collisions are, using `restitution_coefficient` (0 for no bounce, 1 for perfect bounce).
*   **`<contact>`**: Can be used for more advanced contact properties, including `collide_bitmask` for selective collision detection between groups of links.

## Dynamics: Mass, Inertia, and Joints

The dynamic behavior of your robot depends heavily on its mass, inertia, and the properties of its joints.

### Mass and Inertia (`<inertial>`)

Each `<link>` must have an `<inertial>` element for realistic physics simulation.

*   **`<mass>`**: The mass of the link in kilograms.
*   **`<inertia>`**: A 3x3 inertia tensor matrix, describing how mass is distributed around the link's center of mass. This determines how difficult it is to rotate the link about different axes.
*   **`<origin>`**: The center of mass of the link, relative to the link's origin.

```xml
<link name="wheel_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
  <!-- ... visual and collision ... -->
</link>
```
Accurate mass and inertia properties are crucial. For complex shapes, you might use CAD software to calculate these, or approximate them.

### Joint Dynamics (`<joint>`)

Joint properties significantly influence a robot's dynamics:

*   **`<limit>`**: For revolute and prismatic joints, limits define:
    *   `lower` and `upper`: The minimum and maximum joint positions.
    *   `effort`: The maximum torque/force the joint can exert.
    *   `velocity`: The maximum velocity of the joint.
*   **`<dynamics>`**: Defines friction and damping at the joint.
    *   `friction`: A constant friction term.
    *   `damping`: A velocity-proportional damping term.

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

## Physics Engines

Gazebo can utilize different physics engines. The most common are:

*   **ODE (Open Dynamics Engine)**: The default and most widely used engine in Gazebo, known for its stability and performance.
*   **Bullet**: A popular engine, often used for games and animation, known for good collision detection.
*   **DART (Dynamic Animation and Robotics Toolkit)**: Optimized for robotics research, particularly for motion planning and control.
*   **Simbody**: A high-performance, open-source physics library.

The choice of physics engine can impact the accuracy and performance of your simulation. You can specify the physics engine in your `.world` file:

```xml
<physics type="ode">
  <!-- ODE specific parameters -->
</physics>
```

## Conclusion

Mastering the configuration of gravity, collision properties, and the dynamic characteristics of links and joints is fundamental to creating effective and realistic physics simulations in Gazebo. Accurate physics models ensure that your robot's behavior in simulation closely mirrors its real-world counterpart, allowing for reliable testing and development of control algorithms. The next chapter will focus on defining full robot and environment descriptions using URDF and SDF.
