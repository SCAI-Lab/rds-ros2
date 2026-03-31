#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rds_msgs.msg import PedestrianTracks, PedestrianTrack
import math


class FakePedestrianPublisher(Node):
    def __init__(self):
        super().__init__('fake_pedestrian_publisher')

        self.declare_parameter('topic', 'rds/input/pedestrian_tracks')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('pedestrian_radius', 0.3)

        topic = self.get_parameter('topic').value
        rate  = self.get_parameter('rate').value
        self.radius = self.get_parameter('pedestrian_radius').value

        self.publisher_ = self.create_publisher(PedestrianTracks, topic, 10)
        self.create_timer(1.0 / rate, self.timer_callback)

        self.t = 0.0
        self.dt = 1.0 / rate

        self.get_logger().info(f'Fake pedestrian publisher started on {topic}')

    def timer_callback(self):
        msg = PedestrianTracks()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'rds_frame'

        # --- Pedestrian 1: walks straight toward the robot, resets at 0.8 m ---
        # Starts at (0, 4) in rds_frame and walks toward origin at -0.8 m/s
        y1 = 4.0 - 0.8 * (self.t % 8.0)   # resets before reaching robot (min y ~ 0.8 m)
        p1 = PedestrianTrack()
        p1.track_id = 1
        p1.x  =  0.0
        p1.y  =  max(y1, 0.8)
        p1.vx =  0.0
        p1.vy = -0.8
        p1.radius = self.radius
        msg.tracks.append(p1)

        # --- Pedestrian 2: crosses left to right at y = 2 m ahead ---
        p2 = PedestrianTrack()
        p2.track_id = 2
        p2.x  = -3.0 + 0.6 * (self.t % 10.0)   # resets every 10 s
        p2.y  =  2.0
        p2.vx =  0.6
        p2.vy =  0.0
        p2.radius = self.radius
        msg.tracks.append(p2)

        # --- Pedestrian 3: orbits at 3 m radius (unpredictable) ---
        angle = self.t * 0.4
        orbit_r = 3.0
        speed   = orbit_r * 0.4          # tangential speed
        p3 = PedestrianTrack()
        p3.track_id = 3
        p3.x  =  orbit_r * math.cos(angle)
        p3.y  =  orbit_r * math.sin(angle)
        p3.vx = -speed   * math.sin(angle)
        p3.vy =  speed   * math.cos(angle)
        p3.radius = self.radius
        msg.tracks.append(p3)

        self.publisher_.publish(msg)
        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = FakePedestrianPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
