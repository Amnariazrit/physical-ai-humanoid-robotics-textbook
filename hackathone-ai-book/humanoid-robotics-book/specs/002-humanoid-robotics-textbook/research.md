# Research: Dependency Versioning for Robotics Textbook

## Decision

For the "Physical AI & Humanoid Robotics Textbook", the following dependency versions will be used:

- **ROS 2**: Humble Hawksbill (LTS)
- **Gazebo**: Ignition Gazebo Fortress (LTS)
- **NVIDIA Isaac Sim**: The latest official release at the time of content creation.
- **Python**: 3.10 (as required by ROS 2 Humble)

## Rationale

The primary goal of this research was to identify a set of core dependencies (ROS 2, Gazebo, Isaac Sim) that are stable, compatible, and have long-term support, which is crucial for a textbook that needs to remain relevant and reproducible.

- **ROS 2 Humble**: As a Long-Term Support (LTS) release, Humble provides stability and a longer support window, which is ideal for educational materials. It is widely adopted in the robotics community, ensuring that students are learning on a platform they will encounter professionally. Isaac Sim has official support for ROS 2 Humble.

- **Ignition Gazebo Fortress**: Fortress is also an LTS release of the next-generation Gazebo simulator (Ignition). The robotics community is transitioning from classic Gazebo to Ignition, so teaching with the newer version is more forward-looking. The `ros_gz_bridge` provides the necessary link to ROS 2.

- **Compatibility**:
    - Both ROS 2 Humble and Ignition Gazebo Fortress are designed to work together.
    - NVIDIA Isaac Sim officially supports ROS 2 Humble and provides a robust ROS 2 bridge.
    - While Isaac Sim uses a newer version of Python (3.11) than ROS 2 Humble (3.10), it ships with a pre-compiled `rclpy` library to handle this discrepancy, ensuring that ROS 2 nodes can communicate with Isaac Sim. The textbook will need to clearly document the setup for this interoperability.
    - NVIDIA has also been working on connectors between Gazebo and Omniverse (the platform for Isaac Sim), which could be a topic for an advanced chapter.

## Alternatives Considered

- **ROS 2 Jazzy**: While newer and supported by Isaac Sim, Jazzy is not an LTS release. For a textbook, prioritizing stability over the newest features is more important.
- **Classic Gazebo**: While it has been the standard for a long time, it is now in maintenance mode, and the community is shifting to Ignition Gazebo. Teaching with classic Gazebo would be less beneficial for students in the long run.
- **Using a non-LTS Gazebo release**: This would introduce a shorter support lifespan and potential for breaking changes, which is not ideal for a textbook.
