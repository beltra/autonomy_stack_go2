#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    print("Error: You must run this script using python3.")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import time
import os
import json

class Go2SportModeSniffer(Node):
    def __init__(self):
        super().__init__('go2_sport_mode_sniffer')
        self.active_subscriptions = {}
        
        # Check for the sport mode topic periodically
        self.discovery_timer = self.create_timer(2.0, self.discover_and_subscribe)

    def discover_and_subscribe(self):
        for topic_name, topic_types in self.get_topic_names_and_types():
            # Filter ONLY for the sport mode request topic (handles namespaces if present)
            if not topic_name.endswith('/api/sport/request'):
                continue

            if topic_name not in self.active_subscriptions and topic_types:
                try:
                    # Dynamically load unitree_api/msg/Request
                    msg_class = get_message(topic_types[0])
                    if msg_class:
                        cb = lambda msg, t=topic_name: self.message_callback(msg, t)
                        self.active_subscriptions[topic_name] = self.create_subscription(
                            msg_class, topic_name, cb, 10
                        )
                        print(f"[*] Sniffing Go2 Sport Mode on: {topic_name}")
                except Exception as e:
                    print(f"[!] Found {topic_name} but could not load Unitree message type: {e}")

    def message_callback(self, msg, topic_name):
        try:
            # Go2 Sport Mode requests use an api_id to define the action
            api_id = msg.header.identity.api_id
            param_str = msg.parameter
            
            # Map known Unitree Go2 API IDs to human-readable actions
            action_map = {
                1001: "Damp (Emergency Stop)",
                1002: "BalanceStand",
                1004: "StandUp",
                1005: "StandDown",
                1008: "Move (Velocity/Yaw)",
                1011: "SwitchGait",
                1015: "SpeedLevel",
                1030: "FrontFlip",
                1042: "LeftFlip",
                1043: "RightFlip",
                1044: "Backflip",
            }
            action_name = action_map.get(api_id, f"Unknown Action ID: {api_id}")

            timestamp = time.strftime('%H:%M:%S')
            
            print(f"[{timestamp}] 🐕 \033[96m{action_name}\033[0m")
            
            # Print and format the JSON payload (which contains vx, vy, vyaw for movement)
            if param_str:
                try:
                    parsed_json = json.loads(param_str)
                    print(f"    └─ Payload: {json.dumps(parsed_json)}")
                except json.JSONDecodeError:
                    print(f"    └─ Payload: {param_str}")
            print("-" * 55)
            
        except AttributeError:
            print(f"[{time.strftime('%H:%M:%S')}] Unrecognized message structure: {str(msg)[:100]}")


def main(args=None):
    rclpy.init(args=args)
    sniffer = Go2SportModeSniffer()
    
    try:
        os.system('clear' if os.name == 'posix' else 'cls')
        print("=== Unitree Go2 Sport Mode Sniffer ===")
        print("Waiting for movement requests on '/api/sport/request'...")
        print("-" * 65)
        rclpy.spin(sniffer)
    except KeyboardInterrupt:
        print("\nStopping sniffer...")
    finally:
        sniffer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()