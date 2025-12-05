#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json # For parsing simulated object data

class LLMPlannerInterface(Node):
    def __init__(self):
        super().__init__('llm_planner_interface')
        self.plan_publisher = self.create_publisher(String, 'robot_planner/plan_sequence', 10)
        self.voice_command_subscription = self.create_subscription(
            String,
            'voice_commands/text',
            self.voice_command_callback,
            10)
        self.perception_subscription = self.create_subscription(
            String, # Simulating DetectedObject as String
            'perception/detected_objects',
            self.perception_callback,
            10)
        
        self.current_voice_command = ""
        self.detected_objects = []
        self.get_logger().info('LLM Planner Interface Node started.')

    def voice_command_callback(self, msg):
        self.get_logger().info(f'Received voice command: "{msg.data}"')
        self.current_voice_command = msg.data
        self.generate_plan()

    def perception_callback(self, msg):
        self.get_logger().info(f'Received perception data: "{msg.data}"')
        # In a real scenario, this would be parsing a custom msg type
        # For simulation, we'll try to extract object name and position
        try:
            # Example format: "DetectedObject: name=red_block, x=0.5, y=0.1, z=0.0"
            parts = msg.data.split(': ', 1)[1].split(', ')
            obj_data = {p.split('=')[0]: p.split('=')[1] for p in parts}
            self.detected_objects.append(obj_data)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse simulated object data: {e}")
        self.generate_plan()

    def generate_plan(self):
        if not self.current_voice_command:
            return

        plan = []
        lower_command = self.current_voice_command.lower()

        if "move forward to the kitchen" in lower_command:
            plan.append("move_to(kitchen)")
        elif "pick up the red block" in lower_command:
            red_block_detected = False
            red_block_position = None
            for obj in self.detected_objects:
                if obj.get('name') == 'red_block':
                    red_block_detected = True
                    red_block_position = f"x={obj.get('x')},y={obj.get('y')},z={obj.get('z')}"
                    break
            
            if red_block_detected:
                plan.append(f"grasp(red_block, {red_block_position})")
            else:
                self.get_logger().warn("Red block not detected for 'pick up' command.")
        elif "open the fridge door" in lower_command:
            plan.append("open(fridge_door)")
        elif "stop moving" in lower_command:
            plan.append("stop()")
        
        if plan:
            plan_msg = String()
            plan_msg.data = json.dumps(plan) # Serialize plan as JSON string
            self.plan_publisher.publish(plan_msg)
            self.get_logger().info(f'Published plan: {plan_msg.data}')
            # Clear current command after planning
            self.current_voice_command = ""
            self.detected_objects = [] # Clear detected objects for next command
        else:
            self.get_logger().info("No plan generated for current command and perception.")


def main(args=None):
    rclpy.init(args=args)
    llm_planner = LLMPlannerInterface()
    rclpy.spin(llm_planner)
    llm_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
