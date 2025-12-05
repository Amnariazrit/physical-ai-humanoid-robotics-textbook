# Glossary of Robotics & AI Terms

This glossary provides definitions for key terms used throughout the "Physical AI & Humanoid Robotics Textbook". It will be expanded to include at least 150 terms related to robotics, artificial intelligence, ROS 2, Gazebo, NVIDIA Isaac Sim, and Vision-Language-Action models.

## A

**Action (ROS 2)**: A high-level, goal-oriented communication mechanism in ROS 2 designed for long-running tasks. It provides a goal, feedback on progress, and a final result, and can be preempted.

**Actuator**: A component of a machine that is responsible for moving or controlling a mechanism or system. In robotics, this often refers to motors that drive joints.

**AI Agent**: An autonomous entity that perceives its environment through sensors and acts upon that environment using actuators. It works to achieve its goals.

**AI-Native**: Refers to systems, software, or workflows designed from the ground up to leverage artificial intelligence capabilities, often integrating AI models directly into core processes.

**Algorithm**: A step-by-step procedure for solving a problem or accomplishing a task.

**Ambient Light**: In 3D graphics and simulation, a type of light that uniformly illuminates all objects in a scene, providing a base level of brightness.

**Autonomous System**: A system that can perform tasks without human intervention. In robotics, this often implies perception, planning, and execution capabilities.

## B

**Backlash**: A mechanical play or looseness in a system's components, often found in gear trains, which can lead to inaccuracies in robot joint positioning.

**Behavior (Robot)**: The observable actions or responses of a robot to its environment or internal states.

**Bipedal Locomotion**: The act of moving on two legs, as seen in humanoids. This is a complex problem in robotics involving balance, gait generation, and stability.

**Body (Physics)**: In physics simulation, a rigid body is an object that does not deform or change shape under external forces.

**Bounding Box**: A rectangular (or cuboidal) box that fully encloses an object in an image or 3D space. Commonly used in object detection.

## C

**CAD (Computer-Aided Design)**: Software used to design and document products, often used to create 3D models of robot components.

**Camera (Simulated)**: A virtual sensor in a simulator that mimics a real camera, providing RGB, depth, or other visual data.

**Capstone Project**: A culminating academic experience for students, integrating knowledge and skills acquired throughout a program or textbook.

**Client (ROS 2)**: In ROS 2, a node that sends a request to a service server or a goal to an action server.

**Collision Detection**: The process of determining if two or more objects in a simulation are intersecting or touching.

**Cognitive Planning**: The process of reasoning about a task and generating a sequence of high-level actions to achieve a goal, often involving symbolic reasoning and world models.

**Colcon**: A command-line tool used to build a set of ROS 2 packages. It's a successor to `catkin`.

**Computer Vision**: A field of artificial intelligence that enables computers and systems to derive meaningful information from digital images, videos, and other visual inputs.

**Control System**: A device or set of devices that manages, commands, directs, or regulates the behavior of other devices or systems. In robotics, it governs robot motion.

**CUDA**: A parallel computing platform and programming model developed by NVIDIA for its GPUs.

**Custom CSS**: Cascading Style Sheets (CSS) code specifically written or modified to customize the appearance of a website beyond its default styling.

## D

**DDS (Data Distribution Service)**: A middleware protocol standard used by ROS 2 for real-time, high-performance, and scalable data exchange in distributed systems.

**Deep Learning**: A subfield of machine learning that uses artificial neural networks with multiple layers (deep neural networks) to learn from data.

**Degrees of Freedom (DOF)**: The number of independent parameters that define the configuration or state of a mechanical system. A robot arm with 6 revolute joints has 6 DOF.

**Deployment**: The process of putting a developed system or model into operation in its target environment (e.g., deploying a trained AI model to a robot).

**Depth Camera**: A camera that captures not only color information but also the distance to objects in its field of view, generating depth images or point clouds.

**Depth Perception**: The ability to perceive the relative distances of objects in a three-dimensional environment.

**Digital Twin**: A virtual representation of a physical object or system, often updated with real-time data, used for simulation, analysis, and monitoring.

**Distributed System**: A system whose components are located on different networked computers, which communicate and coordinate their actions by passing messages.

**Docusaurus**: An open-source static site generator used to build documentation websites, often leveraging Markdown files.

**Domain Randomization (DR)**: A technique used in reinforcement learning and simulation where non-essential parameters of the simulation are randomized during training to improve the transferability of policies to the real world.

**Dynamics (Robot)**: The study of the forces and torques that cause motion in a robot, taking into account mass, inertia, and joint properties.

## E

**Edge Case**: A problem or situation that occurs only at an extreme (minimum or maximum) operating parameter. In software, often used to test robustness.

**Embedded System**: A computer system with a dedicated function within a larger mechanical or electrical system, often with real-time computing constraints.

**Environment (RL)**: The world an RL agent interacts with, providing states, receiving actions, and returning rewards.

## F

**Fast DDS**: An open-source implementation of the DDS standard, often used as the default RMW (ROS Middleware) in ROS 2.

**Feedback (Action)**: Periodic updates provided by a ROS 2 Action Server to an Action Client, indicating the progress of a long-running goal.

**Flesch-Kincaid Grade Level**: A readability test designed to indicate how difficult a reading passage is to understand, expressed as a U.S. school grade level.

**Friction**: The force resisting the relative motion of solid surfaces, fluid layers, and material elements sliding against each other. Important for realistic robot interaction in simulation.

**Functional Requirement**: A requirement that specifies what the system should do, describing its behavior or features.

## G

**Gait Generation**: The process of generating the sequence of steps and leg movements for a multi-legged robot (e.g., a humanoid) to walk or run.

**Gazebo**: A popular 3D robotics simulator, providing accurate physics simulation and extensive sensor modeling.

**Gesture Recognition**: The ability of a system to identify and interpret human gestures (e.g., pointing, hand signals) from visual input.

**Git Repository**: A version control system that tracks changes in source code and other files, allowing multiple contributors to collaborate and manage different versions of a project.

**Glossary**: An alphabetical list of terms in a particular domain of knowledge with the definitions for those terms.

**Goal (Action)**: The request sent by a ROS 2 Action Client to an Action Server to initiate a long-running task.

**GPU (Graphics Processing Unit)**: A specialized electronic circuit designed to rapidly manipulate and alter memory to accelerate the creation of images, videos, and increasingly, AI computations.

**Grasp**: The act of taking hold of an object firmly with one's hand or an end effector.

## H

**Hallucination (AI)**: In AI, especially with LLMs, generating plausible-sounding but incorrect or fabricated information.

**Hardware-in-the-Loop (HIL) Simulation**: A simulation technique where a real hardware component (e.g., a robot controller) is connected to a virtual simulation environment.

**Human-Robot Interaction (HRI)**: The study of interactions between humans and robots, focusing on designing robots that are intuitive and effective for human use.

**Humanoid Robot**: A robot with its overall shape built to resemble the human body.

## I

**Ignition Gazebo**: The modern, modular, and extensible successor to the classic Gazebo simulator.

**IMU (Inertial Measurement Unit)**: An electronic device that measures and reports a body's specific force, angular rate, and sometimes the orientation of the body. Used for robot localization and stabilization.

**Inertia**: A property of matter by which it continues in its existing state of rest or uniform motion in a straight line, unless that state is changed by an external force. In URDF, the inertia tensor describes mass distribution.

**Inverse Kinematics (IK)**: The process of determining the joint parameters (angles or displacements) of a robot's kinematic chain to achieve a desired end-effector position and orientation.

**Isaac Gym**: A high-performance simulation platform within NVIDIA Isaac Sim, designed specifically for parallelized reinforcement learning training.

**Isaac ROS**: A suite of hardware-accelerated ROS 2 packages optimized to run on NVIDIA GPUs, providing high-performance implementations of key robotics algorithms.

**Isaac Sim**: NVIDIA's extensible robotics simulation platform built on Omniverse, providing photorealistic, physically accurate virtual environments.

## J

**Joint (Robot)**: A connection between two links in a robot's kinematic chain, allowing for specific types of relative motion (e.g., rotation, translation).

**Joint Limits**: The physical constraints on the range of motion for a robot's joints.

## K

**Kinematics**: The study of motion without considering the forces that cause it. In robotics, it deals with the spatial configuration of a robot manipulator and its geometric relationships.

## L

**Large Language Model (LLM)**: A deep learning model that can understand and generate human-like text, often used for natural language understanding, generation, and planning.

**Launch File (ROS 2)**: An XML or Python file used in ROS 2 to start and configure multiple nodes, parameters, and other components of a robot system.

**LiDAR (Light Detection and Ranging)**: A remote sensing method that uses pulsed laser to measure variable distances to the Earth. In robotics, used for mapping and obstacle detection.

**Link (Robot)**: A rigid body segment of a robot's kinematic chain, connected by joints.

**Localization**: The process by which a robot determines its own position and orientation within a known environment.

**Locomotion**: The ability to move from one place to another. In humanoids, this refers to walking, running, or other forms of bipedal movement.

## M

**Manipulation (Robot)**: The process of a robot interacting with objects in its environment, typically involving grasping, lifting, and placing.

**Markdown**: A lightweight markup language for creating formatted text using a plain-text editor. Commonly used for documentation.

**Message (ROS 2)**: A data structure used for communication between ROS 2 nodes over topics.

**Message Type (ROS 2)**: Defines the structure and data types of data contained within a ROS 2 message.

**Microcontroller**: A small computer on a single integrated circuit, often used in embedded systems for low-level control of robot hardware.

**Middleware**: Software that provides services to applications beyond those available from the operating system, enabling communication and data management in distributed systems. ROS 2's DDS is a form of middleware.

**Multi-modal Interaction**: Human-computer or human-robot interaction that involves multiple communication modalities, such as speech, gesture, and vision.

## N

**Natural Language Processing (NLP)**: A field of artificial intelligence focused on enabling computers to understand, interpret, and generate human language.

**Natural Language Understanding (NLU)**: A subfield of NLP focused on extracting meaning from human language.

**Navigation (Robot)**: The process of moving a robot from a start location to a goal location while avoiding obstacles.

**Nav2 (ROS 2 Navigation Stack)**: The primary navigation framework for mobile robots in ROS 2, providing global and local planners, controllers, and recovery behaviors.

**Node (ROS 2)**: An executable process in ROS 2 that performs a specific task or computation.

**Non-Functional Requirement (NFR)**: A requirement that specifies criteria for judging the operation of a system, rather than specific behaviors (e.g., performance, security, usability).

## O

**Object Detection**: A computer vision technique that identifies and locates objects within an image or video.

**Object Localization**: The process of determining the precise position of an identified object in 2D or 3D space.

**Omniverse (NVIDIA)**: A platform for 3D design collaboration and simulation, using USD as its core data format. NVIDIA Isaac Sim is built on Omniverse.

**OpenAPI Specification**: A standard, language-agnostic interface description for RESTful APIs, allowing humans and computers to discover and understand the capabilities of a service.

**OpenCV (Open Source Computer Vision Library)**: A widely used open-source library for computer vision and machine learning.

**Open3D**: An open-source library that supports rapid development for 3D data processing, including point cloud operations.

**Operating System (OS)**: Software that manages computer hardware and software resources and provides common services for computer programs.

**Origin (URDF)**: Specifies the position and orientation of a coordinate frame (e.g., for a link, joint, or visual element) relative to its parent frame.

## P

**Parameter (ROS 2)**: Key-value pairs used by ROS 2 nodes to store and retrieve configuration data at runtime.

**Perception (Robot)**: The ability of a robot to sense and interpret its environment using sensors.

**Phase (Tasks)**: A logical grouping of related tasks in an implementation plan, often representing a major milestone.

**Physics Engine**: Software that simulates physical phenomena (e.g., gravity, collisions, friction) in a virtual environment.

**PhysX (NVIDIA)**: A real-time physics engine developed by NVIDIA, used in Isaac Sim for high-fidelity physics simulation.

**PID Controller (Proportional-Integral-Derivative)**: A control loop feedback mechanism widely used in industrial control systems to regulate processes.

**Plan (LLM)**: A sequence of high-level abstract actions generated by an LLM to achieve a specified goal.

**Plagiarism**: The practice of taking someone else's work or ideas and passing them off as one's own.

**Pointing Gesture**: A human gesture where a finger or hand is directed towards an object or location to indicate attention or intent.

**Policy (RL)**: A function that maps states to actions, defining the agent's behavior in the environment.

**Pose**: A combination of position and orientation in 3D space.

**Prismatic Joint**: A type of robot joint that allows linear motion (translation) along a single axis.

**Prompt Engineering**: The process of designing and refining input prompts for large language models to elicit desired outputs.

**PyTorch**: An open-source machine learning framework developed by Facebook's AI Research lab (FAIR).

**Pytest**: A popular Python testing framework used for writing concise and scalable test cases.

**Python**: A high-level, interpreted programming language widely used in robotics and AI.

## R

**RAG (Retrieval-Augmented Generation)**: An AI technique that combines information retrieval with text generation, allowing LLMs to answer questions by first finding relevant information from a knowledge base.

**`rclcpp`**: The C++ client library for ROS 2.

**`rclpy`**: The Python client library for ROS 2.

**Reality Gap**: The discrepancy between the performance of a robot in simulation and its performance in the real world.

**Reinforcement Learning (RL)**: A type of machine learning where an agent learns to make decisions by performing actions in an environment and receiving rewards or penalties.

**Revolute Joint**: A type of robot joint that allows rotational motion around a single axis (like a hinge).

**Rigid Body Dynamics**: The study of the motion of interconnected rigid bodies under the action of external forces.

**Robot Operating System (ROS)**: An open-source, flexible framework for writing robot software. ROS 2 is its successor.

**`robot_state_publisher`**: A ROS 2 node that reads the robot's URDF description and the joint states, then publishes the robot's 3D pose (transformations) as TF messages.

**ROS 2 Package**: A basic unit of organization in ROS 2, containing nodes, launch files, message/service/action definitions, and other resources.

**ROS 2 Topic**: A named data bus in ROS 2 over which nodes can exchange messages using a publish/subscribe communication model.

**RViz2**: A 3D visualization tool for ROS 2 that allows you to view robot models, sensor data, and other ROS 2 messages in a graphical environment.

## S

**SDF (Simulation Description Format)**: An XML format used to describe objects and environments for robot simulators like Gazebo. It is more comprehensive than URDF.

**Semantic Versioning**: A versioning scheme (MAJOR.MINOR.PATCH) widely used in software development to communicate the type of changes in each new release.

**Sensor**: A device that detects and responds to some type of input from the physical environment.

**Sensor Simulation**: The process of creating virtual sensors in a simulator that mimic the behavior and data output of real-world sensors.

**Service (ROS 2)**: A request/reply communication mechanism in ROS 2 for synchronous, one-time interactions between a client and a server node.

**Sim-to-Real Transfer**: The process of transferring a robot policy or model trained in a simulated environment to a real physical robot.

**SLAM (Simultaneous Localization and Mapping)**: The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

**Speech-to-Text (STT)**: The process of converting spoken language into written text.

**`std_msgs`**: A package in ROS 2 that provides common message types for primitive data types (e.g., `String`, `Int32`).

**Subscriber (ROS 2)**: A node that receives messages from a ROS 2 topic.

**Synthetic Data Generation (SDG)**: The process of creating artificial data, often using simulations, to train AI models when real-world data is scarce or expensive.

## T

**Task Decomposition**: The process of breaking down a complex goal into smaller, more manageable sub-goals or actions.

**TensorFlow**: An open-source machine learning framework developed by Google.

**TF (Transform Library)**: A ROS 2 library that keeps track of multiple coordinate frames and allows for transformations between them.

**Textbook**: A comprehensive manual of instruction in any branch of study.

**Topic (ROS 2)**: (See ROS 2 Topic)

## U

**Ubuntu**: A popular Debian-based Linux operating system, widely used in robotics development.

**Unified Robot Description Format (URDF)**: An XML format used in ROS to describe the physical characteristics of a robot.

**Unitree**: A company known for developing quadruped and humanoid robots.

**Unity**: A cross-platform game engine that can be used for 3D visualization and simulation.

## V

**Verification**: The process of evaluating whether a product, service, or system complies with a regulation, requirement, specification, or imposed condition.

**Versioning**: The process of assigning unique version names or numbers to unique states of computer software.

**Visual SLAM (VSLAM)**: SLAM performed using visual sensor data, typically from cameras.

**Vision-Language-Action (VLA) Model**: An AI model that integrates capabilities from computer vision, natural language processing, and robotic action planning to enable robots to understand and act upon human instructions in complex environments.

**Voice Command**: Instructions given to a system through spoken language.

## W

**Whisper (OpenAI)**: A general-purpose speech recognition model developed by OpenAI, capable of transcribing and translating speech.

## X

**Xacro (XML Macros)**: An XML macro language that allows for more modular and reusable URDF files.

## Z

**ZMP (Zero Moment Point)**: A concept used in humanoid robotics to analyze and control dynamic balance during locomotion. It is the point on the ground where the net moment of all forces (gravitational and inertial) acting on the robot is zero.
