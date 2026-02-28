import json
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_api.msg import Request


ROBOT_SPORT_API_ID_STOPMOVE = 1003
ROBOT_SPORT_API_ID_SWITCHGAIT = 1011
ROBOT_SPORT_API_ID_BODYHEIGHT = 1013
ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT = 1014
ROBOT_SPORT_API_ID_CONTINUOUSGAIT = 1019


class MotionExecutor(Node):
    def __init__(self) -> None:
        super().__init__('motion_executor')

        self.declare_parameter('dry_run', True)
        self.declare_parameter('motion_backend', 'legacy_sport_api')
        self.declare_parameter('gait_stairs', 3)
        self.declare_parameter('stair_body_height', 0.05)
        self.declare_parameter('stair_foot_raise_height', 0.08)

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.motion_backend = str(self.get_parameter('motion_backend').value)
        self.gait_stairs = int(self.get_parameter('gait_stairs').value)
        self.stair_body_height = float(self.get_parameter('stair_body_height').value)
        self.stair_foot_raise_height = float(self.get_parameter('stair_foot_raise_height').value)

        self.req_pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.status_pub = self.create_publisher(String, '/stairs/motion_status', 10)
        self.create_subscription(String, '/stairs/motion_command', self.command_callback, 20)

        self.get_logger().info(f'motion_executor started (backend={self.motion_backend}, dry_run={self.dry_run})')

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_request(self, api_id: int, payload: Optional[dict] = None) -> None:
        req = Request()
        req.header.identity.api_id = int(api_id)
        if payload is not None:
            req.parameter = json.dumps(payload)
        if self.dry_run:
            self.get_logger().info(f'[dry-run] req api_id={api_id} payload={payload}')
            return
        self.req_pub.publish(req)

    def execute_legacy(self, command: str) -> None:
        if command == 'PREPARE_ASCEND' or command == 'PREPARE_DESCEND':
            self.publish_request(ROBOT_SPORT_API_ID_SWITCHGAIT, {'data': self.gait_stairs})
            self.publish_request(ROBOT_SPORT_API_ID_BODYHEIGHT, {'data': self.stair_body_height})
            self.publish_request(ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT, {'data': self.stair_foot_raise_height})
            self.publish_request(ROBOT_SPORT_API_ID_CONTINUOUSGAIT, {'data': True})
        elif command == 'ASCEND' or command == 'DESCEND':
            self.publish_request(ROBOT_SPORT_API_ID_CONTINUOUSGAIT, {'data': True})
        elif command == 'EXIT_STAIRS':
            self.publish_request(ROBOT_SPORT_API_ID_STOPMOVE)
            self.publish_request(ROBOT_SPORT_API_ID_CONTINUOUSGAIT, {'data': False})
            self.publish_request(ROBOT_SPORT_API_ID_BODYHEIGHT, {'data': 0.0})
            self.publish_request(ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT, {'data': 0.0})
        elif command == 'IDLE':
            pass

    def execute_motion_v2(self, command: str) -> None:
        self.get_logger().warn(
            f'Motion Services Interface V2 backend selected but concrete stair API mapping is pending; command={command}'
        )

    def command_callback(self, msg: String) -> None:
        command = msg.data
        self.publish_status(f'EXEC:{command}')
        if self.motion_backend == 'motion_services_v2':
            self.execute_motion_v2(command)
        else:
            self.execute_legacy(command)


def main() -> None:
    rclpy.init()
    node = MotionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()