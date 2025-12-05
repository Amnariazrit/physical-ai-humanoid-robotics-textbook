#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# For actual Whisper integration, you'd install the 'openai-whisper' package
# and potentially use an audio capture library like 'pyaudio'.
# For this example, we'll simulate the transcription.

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher_ = self.create_publisher(String, 'voice_commands/text', 10)
        self.timer = self.create_timer(5.0, self.simulate_voice_command) # Simulate a command every 5 seconds
        self.get_logger().info('Voice Command Node started. Simulating commands...')

        self.simulated_commands = [
            "Robot, move forward to the kitchen.",
            "Please pick up the red block.",
            "Can you open the fridge door?",
            "Robot, what is your battery level?",
            "Stop moving."
        ]
        self.command_index = 0

    def simulate_voice_command(self):
        # In a real scenario, this would involve:
        # 1. Capturing audio from a microphone.
        # 2. Passing the audio to the Whisper model for transcription.
        # 3. Getting the transcribed text.

        # Simulate Whisper transcription output
        if self.command_index < len(self.simulated_commands):
            transcribed_text = self.simulated_commands[self.command_index]
            self.command_index += 1
        else:
            transcribed_text = "No new command."
            self.get_logger().info('All simulated commands sent. Looping...')
            self.command_index = 0 # Loop the commands for continuous simulation

        msg = String()
        msg.data = transcribed_text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}" (Simulated Whisper output)')


def main(args=None):
    rclpy.init(args=args)
    voice_command_node = VoiceCommandNode()
    rclpy.spin(voice_command_node)
    voice_command_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
