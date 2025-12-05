import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci # Correct import path


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            sequence.append(sequence[i] + sequence[i-1])
            feedback_msg = Fibonacci.Feedback()
            feedback_msg.sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.sequence))
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info('Result: {0}'.format(result.sequence))
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)

    fibonacci_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
