import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String


class StairDetectorLidarImu(Node):
    def __init__(self) -> None:
        super().__init__('stair_detector_lidar_imu')

        self.declare_parameter('stair_pitch_ahead_deg', 8.0)
        self.declare_parameter('stair_pitch_on_deg', 14.0)
        self.declare_parameter('clear_hold_sec', 1.2)

        self.stair_pitch_ahead_deg = float(self.get_parameter('stair_pitch_ahead_deg').value)
        self.stair_pitch_on_deg = float(self.get_parameter('stair_pitch_on_deg').value)
        self.clear_hold_sec = float(self.get_parameter('clear_hold_sec').value)

        self.state_pub = self.create_publisher(String, '/stairs/state', 10)
        self.confidence_pub = self.create_publisher(Float32, '/stairs/confidence', 10)

        self.create_subscription(Odometry, '/state_estimation', self.odom_callback, 20)

        self.last_stairs_time = self.get_clock().now()
        self.last_state = 'NO_STAIRS'

        self.get_logger().info('stair_detector_lidar_imu started (MVP L1/IMU mode)')

    def odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        pitch_deg = abs(math.degrees(pitch))

        now = self.get_clock().now()
        state = 'NO_STAIRS'
        confidence = 0.0

        if pitch_deg >= self.stair_pitch_on_deg:
            state = 'ON_STAIRS'
            confidence = min(1.0, pitch_deg / (self.stair_pitch_on_deg + 8.0))
            self.last_stairs_time = now
        elif pitch_deg >= self.stair_pitch_ahead_deg:
            state = 'STAIRS_AHEAD'
            confidence = min(1.0, pitch_deg / self.stair_pitch_on_deg)
            self.last_stairs_time = now
        else:
            elapsed = (now - self.last_stairs_time).nanoseconds / 1e9
            if elapsed < self.clear_hold_sec:
                state = 'STAIRS_CLEAR'
                confidence = max(0.1, 1.0 - elapsed / self.clear_hold_sec)

        if state != self.last_state:
            self.get_logger().info(f'stairs state: {self.last_state} -> {state} (pitch={pitch_deg:.1f} deg)')
            self.last_state = state

        out_state = String()
        out_state.data = state
        self.state_pub.publish(out_state)

        out_conf = Float32()
        out_conf.data = float(confidence)
        self.confidence_pub.publish(out_conf)


def main() -> None:
    rclpy.init()
    node = StairDetectorLidarImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()