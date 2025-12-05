# Quickstart Guide: Humanoid Robotics Textbook

This guide provides the essential steps to get your environment set up to follow along with the "Physical AI & Humanoid Robotics Textbook".

## Prerequisites

- A computer running **Ubuntu 22.04**.
- A compatible **NVIDIA GPU** for the NVIDIA Isaac Sim modules.
- Familiarity with basic Linux command-line operations and Python.

## 1. Clone the Repository

First, clone the textbook's Git repository to your local machine:

```bash
git clone https://github.com/Amnariazrit/physical-ai-humanoid-robotics-textbook.git
cd physical-ai-humanoid-robotics-textbook
```

## 2. Install Core Dependencies

The textbook relies on ROS 2 Humble and Ignition Gazebo Fortress.

### Install ROS 2 Humble

Follow the official ROS 2 documentation to install ROS 2 Humble Hawksbill on your Ubuntu 22.04 system.

[**ROS 2 Humble Installation Guide**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Make sure to also install `colcon` and other development tools as recommended in the guide.

### Install Ignition Gazebo Fortress

Install Gazebo Fortress, which is the version compatible with ROS 2 Humble.

```bash
sudo apt-get update
sudo apt-get install ignition-fortress
```

You will also need the ROS 2 to Gazebo bridge:
```bash
sudo apt-get install ros-humble-ros-gz
```

## 3. Set up NVIDIA Isaac Sim

For the modules involving advanced simulation and AI, you will need NVIDIA Isaac Sim.

1.  Download and install **NVIDIA Omniverse Launcher**.
2.  From the launcher, install the latest version of **NVIDIA Isaac Sim**.
3.  Follow the Isaac Sim documentation for "ROS & ROS 2 Installation" to ensure the ROS 2 bridge is set up correctly. This is a critical step for compatibility.

## 4. Build and View the Textbook

The textbook is built using Docusaurus.

### Install Node.js and Yarn

Docusaurus requires Node.js.
```bash
sudo apt-get install nodejs npm
sudo npm install -g yarn
```

### Install Dependencies and Run

Navigate to the root of the cloned repository and install the Docusaurus dependencies:

```bash
yarn install
```

Then, start the local development server:

```bash
yarn start
```

This will open a live-preview of the textbook in your web browser, typically at `http://localhost:3000`. The site will automatically reload as you make changes to the Markdown files.
