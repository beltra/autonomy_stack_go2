import math
from enum import Enum

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class SupervisorMode(str, Enum):
    IDLE = 'IDLE'
    PREPARE_ASCEND = 'PREPARE_ASCEND'
    PREPARE_DESCEND = 'PREPARE_DESCEND'
    ASCEND = 'ASCEND'
    DESCEND = 'DESCEND'
    EXIT_STAIRS = 'EXIT_STAIRS'


class StairSupervisor(Node):
    def __init__(self) -> None:
        super().__init__('stair_supervisor')

        self.declare_parameter('enable_supervisor', True)
        self.declare_parameter('direction_pitch_threshold_deg', 3.0)
        self.declare_parameter('invert_direction_sign', False)
        self.declare_parameter('command_cooldown_sec', 1.0)

        self.enable_supervisor = bool(self.get_parameter('enable_supervisor').value)
        self.direction_pitch_threshold_deg = float(self.get_parameter('direction_pitch_threshold_deg').value)
        self.invert_direction_sign = bool(self.get_parameter('invert_direction_sign').value)
        self.command_cooldown_sec = float(self.get_parameter('command_cooldown_sec').value)

        self.current_stair_state = 'NO_STAIRS'
        self.current_pitch = 0.0
        self.last_direction_ascending = True
        self.mode = SupervisorMode.IDLE
        self.last_command_time = self.get_clock().now()

        self.hold_pub = self.create_publisher(Bool, '/stairs/hold_path_follower', 10)
        self.command_pub = self.create_publisher(String, '/stairs/motion_command', 10)
        self.active_pub = self.create_publisher(Bool, '/stairs/active', 10)

        self.create_subscription(String, '/stairs/state', self.stair_state_callback, 20)
        self.create_subscription(Odometry, '/state_estimation', self.odom_callback, 20)
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info('stair_supervisor started')

    def stair_state_callback(self, msg: String) -> None:
        self.current_stair_state = msg.data

    def odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.current_pitch = float(math.asin(sinp))

    def is_ascending(self) -> bool:
        pitch = self.current_pitch
        if self.invert_direction_sign:
            pitch = -pitch
        threshold_rad = self.direction_pitch_threshold_deg * 3.1415926 / 180.0
        if abs(pitch) <= threshold_rad:
            return self.last_direction_ascending
        self.last_direction_ascending = pitch >= 0.0
        return self.last_direction_ascending

    def publish_hold(self, hold: bool) -> None:
        msg = Bool()
        msg.data = hold
        self.hold_pub.publish(msg)

        active = Bool()
        active.data = hold
        self.active_pub.publish(active)

    def maybe_publish_command(self, mode: SupervisorMode) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_command_time).nanoseconds / 1e9
        if dt < self.command_cooldown_sec and mode == self.mode:
            return

        self.mode = mode
        self.last_command_time = now
        msg = String()
        msg.data = mode.value
        self.command_pub.publish(msg)

    def tick(self) -> None:
        if not self.enable_supervisor:
            self.publish_hold(False)
            self.maybe_publish_command(SupervisorMode.IDLE)
            return

        state = self.current_stair_state
        if state in ('STAIRS_AHEAD', 'ON_STAIRS'):
            self.publish_hold(True)
            if state == 'STAIRS_AHEAD':
                if self.is_ascending():
                    self.maybe_publish_command(SupervisorMode.PREPARE_ASCEND)
                else:
                    self.maybe_publish_command(SupervisorMode.PREPARE_DESCEND)
            else:
                if self.is_ascending():
                    self.maybe_publish_command(SupervisorMode.ASCEND)
                else:
                    self.maybe_publish_command(SupervisorMode.DESCEND)
        elif state == 'STAIRS_CLEAR':
            self.publish_hold(True)
            self.maybe_publish_command(SupervisorMode.EXIT_STAIRS)
        else:
            self.publish_hold(False)
            self.maybe_publish_command(SupervisorMode.IDLE)


def main() -> None:
    rclpy.init()
    node = StairSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()