#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Assuming these are custom action messages defined in a package 'capstone_interfaces'
# For this example, we place the .action files in the same directory.
# In a real setup, you'd build a ROS 2 package for these interfaces.
from static.code_examples.module4.ch5_capstone_integration.t034_4_action_servers.action import MoveToLocation


class MoveToLocationActionServer(Node):

    def __init__(self):
        super().__init__('move_to_location_action_server')
        self._action_server = ActionServer(
            self,
            MoveToLocation,
            'move_to_location',
            self.execute_callback)
        self.get_logger().info('MoveToLocation Action Server started.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: Move to {goal_handle.request.location_name}')

        # Simulate robot movement
        feedback_msg = MoveToLocation.Feedback()
        feedback_msg.current_status = f"Moving towards {goal_handle.request.location_name}..."
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f'Feedback: {feedback_msg.current_status}')

        # In a real robot, this would involve motion planning and execution
        # Simulate completion
        await self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2)) # Simulate delay

        goal_handle.succeed()

        result = MoveToLocation.Result()
        result.success = True
        self.get_logger().info(f'Move to {goal_handle.request.location_name} completed successfully.')
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = MoveToLocationActionServer()
    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
