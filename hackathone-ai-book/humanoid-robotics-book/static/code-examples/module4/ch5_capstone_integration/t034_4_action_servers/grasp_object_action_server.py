#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Assuming these are custom action messages defined in a package 'capstone_interfaces'
from static.code_examples.module4.ch5_capstone_integration.t034_4_action_servers.action import GraspObject


class GraspObjectActionServer(Node):

    def __init__(self):
        super().__init__('grasp_object_action_server')
        self._action_server = ActionServer(
            self,
            GraspObject,
            'grasp_object',
            self.execute_callback)
        self.get_logger().info('GraspObject Action Server started.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: Grasp {goal_handle.request.object_name} at {goal_handle.request.object_position}')

        # Simulate robot grasping
        feedback_msg = GraspObject.Feedback()
        feedback_msg.current_status = f"Attempting to grasp {goal_handle.request.object_name}..."
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f'Feedback: {feedback_msg.current_status}')

        # In a real robot, this would involve inverse kinematics, gripper control, force sensing
        # Simulate completion
        await self.get_clock().sleep_for(rclpy.duration.Duration(seconds=3)) # Simulate delay

        goal_handle.succeed()

        result = GraspObject.Result()
        result.success = True
        result.grasped_object_name = goal_handle.request.object_name
        self.get_logger().info(f'Grasped {goal_handle.request.object_name} successfully.')
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = GraspObjectActionServer()
    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
