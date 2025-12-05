#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import json
import time

# Assuming these are custom action messages defined in a package 'capstone_interfaces'
# For this example, we place the .action files in the same directory.
# In a real setup, you'd build a ROS 2 package for these interfaces.
from static.code_examples.module4.ch5_capstone_integration.t034_4_action_servers.action import MoveToLocation, GraspObject, OpenDoor, CloseDoor


class ActionSequencerNode(Node):

    def __init__(self):
        super().__init__('action_sequencer_node')
        self.plan_subscription = self.create_subscription(
            String,
            'robot_planner/plan_sequence',
            self.plan_callback,
            10)
        self.plan_subscription  # prevent unused variable warning

        self.callback_group = ReentrantCallbackGroup()

        # Action Clients for each skill
        self._move_client = ActionClient(self, MoveToLocation, 'move_to_location', callback_group=self.callback_group)
        self._grasp_client = ActionClient(self, GraspObject, 'grasp_object', callback_group=self.callback_group)
        self._open_client = ActionClient(self, OpenDoor, 'open_door', callback_group=self.callback_group)
        self._close_client = ActionClient(self, CloseDoor, 'close_door', callback_group=self.callback_group)

        # Map action names to clients
        self.action_clients = {
            'move_to': self._move_client,
            'grasp': self._grasp_client,
            'open': self._open_client,
            'close': self._close_client,
        }

        self.current_plan_sequence = []
        self.current_action_index = 0
        self.executing_action = False
        self.get_logger().info('Action Sequencer Node started. Awaiting plans...')

    def plan_callback(self, msg):
        if self.executing_action:
            self.get_logger().warn("Received new plan while executing previous. Ignoring for now.")
            return

        self.current_plan_sequence = json.loads(msg.data) # Deserialize JSON string back to list
        self.current_action_index = 0
        self.get_logger().info(f"Received new plan: {self.current_plan_sequence}")
        self.execute_next_action()

    def execute_next_action(self):
        if self.current_action_index >= len(self.current_plan_sequence):
            self.get_logger().info("Plan execution complete.")
            self.executing_action = False
            return

        action_str = self.current_plan_sequence[self.current_action_index]
        self.get_logger().info(f"Executing action: {action_str}")
        self.executing_action = True

        # Parse action string (e.g., "move_to(kitchen)" or "grasp(red_block, x=0.5,y=0.1,z=0.0)")
        action_name_full = action_str.split('(')[0]
        params_str = action_str.split('(')[1].strip(')')

        action_name = action_name_full.split('_', 1)[0] # e.g., 'move', 'grasp'

        if action_name_full in self.action_clients: # Direct match for skill name
            client = self.action_clients[action_name_full]
            
            # Wait for action server
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"Action server for {action_name_full} not available. Aborting plan.")
                self.executing_action = False
                return

            goal_msg = None
            if action_name_full == 'move_to':
                goal_msg = MoveToLocation.Goal()
                goal_msg.location_name = params_str
            elif action_name_full == 'grasp':
                goal_msg = GraspObject.Goal()
                parts = params_str.split(',', 1) # Split object name and position
                goal_msg.object_name = parts[0].strip()
                if len(parts) > 1:
                    goal_msg.object_position = parts[1].strip() # Example: "x=0.5,y=0.1,z=0.0"
                else:
                    goal_msg.object_position = "" # No position given
            elif action_name_full == 'open':
                goal_msg = OpenDoor.Goal()
                goal_msg.door_name = params_str
            elif action_name_full == 'close':
                goal_msg = CloseDoor.Goal()
                goal_msg.door_name = params_str
            
            if goal_msg:
                self._send_goal_future = client.send_goal_async(goal_msg)
                self._send_goal_future.add_done_callback(self.goal_response_callback)
            else:
                self.get_logger().error(f"Failed to create goal message for action: {action_name_full}. Aborting.")
                self.executing_action = False
        else:
            self.get_logger().error(f"No action client found for skill: {action_name_full}. Aborting plan.")
            self.executing_action = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected by server: {goal_handle.goal_id}')
            self.executing_action = False
            return

        self.get_logger().info(f'Goal accepted by server: {goal_handle.goal_id}')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Action completed successfully.")
            self.current_action_index += 1
            # Wait a moment before next action for simulation realism
            time.sleep(0.5)
            self.execute_next_action() # Proceed to the next action in the plan
        else:
            self.get_logger().error(f"Action failed. Aborting plan.")
            self.executing_action = False


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    action_sequencer_node = ActionSequencerNode()
    executor.add_node(action_sequencer_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        action_sequencer_node.get_logger().info("Action sequencer stopped.")
    finally:
        executor.shutdown()
        action_sequencer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
