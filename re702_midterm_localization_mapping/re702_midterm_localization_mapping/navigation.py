#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
import time
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions


class Navigator(Node):
    def __init__(self):
        super().__init__('delivery_navigator')

        self.audio_pub = self.create_publisher(AudioNoteVector, '/cmd_audio', 10)
        self.navigator = TurtleBot4Navigator()

    def buzzer(self, times: int = 1):
        msg = AudioNoteVector()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.append = False

        notes = []
        for _ in range(times):
            notes.append(AudioNote(
                frequency=440,
                max_runtime=Duration(sec=3, nanosec=500000000)
            ))
            notes.append(AudioNote(
                frequency=0,
                max_runtime=Duration(sec=0, nanosec=300000000)
            ))

        msg.notes = notes
        self.audio_pub.publish(msg)
        self.get_logger().info(f'Beep {times}x sent to /cmd_audio')

    def execute_navigation(self):

        self.navigator.waitUntilNav2Active()

        pick_pose = self.navigator.getPoseStamped([11.1, -18.3], TurtleBot4Directions.SOUTH)
        place_pose = self.navigator.getPoseStamped([-10.3, 11.7], TurtleBot4Directions.EAST)

        self.get_logger().info('To the pick location...')
        self.navigator.startToPose(pick_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Arrived at pick location')
        self.buzzer(times=1)
        time.sleep(5)

        self.get_logger().info('To the place location...')
        self.navigator.startToPose(place_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Arrived at place location')
        self.buzzer(times=2)
        time.sleep(5)
        
        self.get_logger().info('Task completed.')


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        node.execute_navigation()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
