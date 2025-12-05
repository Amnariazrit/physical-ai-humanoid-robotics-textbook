#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # For simplicity, we'll use String for DetectedObject and HumanPose

# In a real scenario, you'd define custom message types like:
# from capstone_interfaces.msg import DetectedObject, HumanPose

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.object_publisher = self.create_publisher(String, 'perception/detected_objects', 10)
        self.pose_publisher = self.create_publisher(String, 'perception/human_pose', 10)
        
        self.timer = self.create_timer(3.0, self.simulate_perception) # Simulate perception every 3 seconds
        self.get_logger().info('Perception Node started. Simulating detections...')

        self.simulated_objects = [
            "DetectedObject: name=red_block, x=0.5, y=0.1, z=0.0",
            "DetectedObject: name=green_ball, x=0.8, y=-0.3, z=0.0",
            "DetectedObject: name=fridge, x=1.0, y=0.0, z=1.0"
        ]
        self.simulated_poses = [
            "HumanPose: head_x=0.2, head_y=0.5, pointing_x=0.6, pointing_y=0.2",
            "HumanPose: head_x=0.3, head_y=0.4, pointing_x=0.7, pointing_y=0.1"
        ]
        self.obj_index = 0
        self.pose_index = 0

    def simulate_perception(self):
        # Simulate object detection
        obj_msg = String()
        obj_msg.data = self.simulated_objects[self.obj_index]
        self.object_publisher.publish(obj_msg)
        self.get_logger().info(f'Published: "{obj_msg.data}" (Simulated Object Detection)')
        self.obj_index = (self.obj_index + 1) % len(self.simulated_objects)

        # Simulate human pose detection
        pose_msg = String()
        pose_msg.data = self.simulated_poses[self.pose_index]
        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f'Published: "{pose_msg.data}" (Simulated Human Pose)')
        self.pose_index = (self.pose_index + 1) % len(self.simulated_poses)


def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()
    rclpy.spin(perception_node)
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
