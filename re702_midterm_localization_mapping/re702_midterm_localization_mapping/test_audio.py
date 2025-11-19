#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
import time


class AudioTest(Node):
    def __init__(self):
        super().__init__('audio_test')

        self.audio_pub = self.create_publisher(AudioNoteVector, '/cmd_audio', 10)

    def play_beep(self, times: int = 1):
        msg = AudioNoteVector()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.append = False

        notes = []
        for i in range(times):
            notes.append(AudioNote(
                frequency=440,
                max_runtime=Duration(sec=0, nanosec=500000000)  # 0.5 detik
            ))
            notes.append(AudioNote(
                frequency=0,
                max_runtime=Duration(sec=0, nanosec=300000000)
            ))

        msg.notes = notes
        self.audio_pub.publish(msg)

    def run_test(self):
        
        self.get_logger().info("TEST AUDIO DIMULAI ...")
        
        self.play_beep(1)
        time.sleep(5)

        self.play_beep(2)
        time.sleep(5)

        self.play_beep(3)
        time.sleep(5)
        
        self.play_beep(4)
        time.sleep(5)

        self.get_logger().info("TEST AUDIO SELESAI.")


def main(args=None):
    rclpy.init(args=args)
    node = AudioTest()
    
    node.run_test()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
