#!/usr/bin/env python3
"""
Graph File Bridge + HTTP static file server for FAR Web GUI.

Functions:
  - Serves static web files (HTML/JS/CSS) over HTTP.
  - Provides a ROS2 service /web_gui/download_graph that triggers
    /request_graph_service, waits for the graph on /decoded_vgraph,
    serializes it to VGH text and returns it as a string.
  - Provides a ROS2 topic /web_gui/upload_graph (std_msgs/String)
    that receives VGH text content, writes a temp file, publishes
    the path on /read_file_dir, then deletes the temp file.
"""

import os
import sys
import json
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Bool, Empty
from std_srvs.srv import Trigger
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from visibility_graph_msg.msg import Graph

from http.server import HTTPServer, SimpleHTTPRequestHandler
from functools import partial


class GraphFileBridge(Node):
    def __init__(self):
        super().__init__('graph_file_bridge')

        # Parameters
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('web_dir', '')

        http_port_val = self.get_parameter('http_port').value
        self.http_port = int(http_port_val) if http_port_val else 8080
        web_dir = self.get_parameter('web_dir').value

        if not web_dir or not os.path.isdir(web_dir):
            # Try to find via ament
            try:
                from ament_index_python.packages import get_package_share_directory
                web_dir = os.path.join(
                    get_package_share_directory('far_web_gui'), 'www')
            except Exception:
                web_dir = os.path.join(
                    os.path.dirname(os.path.dirname(__file__)),
                    'share', 'far_web_gui', 'www')

        self.web_dir = web_dir
        self.get_logger().info(f'Serving web files from: {self.web_dir}')

        # Callback groups: allow subscription to fire while service blocks
        self._srv_cb_group = MutuallyExclusiveCallbackGroup()
        self._sub_cb_group = ReentrantCallbackGroup()

        # --- Graph download service ---
        self.download_srv = self.create_service(
            Trigger, '/web_gui/download_graph', self.download_graph_cb,
            callback_group=self._srv_cb_group)

        # Client to request graph from decoder
        self.request_graph_cli = self.create_client(
            Trigger, '/request_graph_service',
            callback_group=self._sub_cb_group)

        # Subscriber for decoded graph (one-shot pattern)
        self._graph_msg = None
        self._graph_event = threading.Event()
        self.graph_sub = self.create_subscription(
            Graph, '/decoded_vgraph', self._graph_cb, 5,
            callback_group=self._sub_cb_group)

        # --- Graph upload topic ---
        self.upload_sub = self.create_subscription(
            String, '/web_gui/upload_graph', self.upload_graph_cb, 5)

        # Publisher to feed file path to decoder's read
        self.read_file_pub = self.create_publisher(String, '/read_file_dir', 5)

        # Start HTTP server in a thread
        self._start_http_server()

        self.get_logger().info(
            f'Graph File Bridge ready. HTTP on port {self.http_port}')

    # ------------------------------------------------------------------
    # Graph download (robot → browser)
    # ------------------------------------------------------------------

    def _graph_cb(self, msg):
        """Cache latest decoded graph message."""
        self._graph_msg = msg
        self._graph_event.set()

    def download_graph_cb(self, request, response):
        """Service: trigger graph request, serialize to VGH, return as string."""
        # 1. Call /request_graph_service
        if not self.request_graph_cli.wait_for_service(timeout_sec=3.0):
            response.success = False
            response.message = 'request_graph_service not available'
            return response

        self._graph_event.clear()
        self._graph_msg = None
        future = self.request_graph_cli.call_async(Trigger.Request())

        # Wait for service response
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 5.0:
            time.sleep(0.05)

        if not future.done():
            response.success = False
            response.message = 'Timeout waiting for request_graph_service'
            return response

        # 2. Wait for graph message on /decoded_vgraph
        if not self._graph_event.wait(timeout=5.0):
            response.success = False
            response.message = 'Timeout waiting for decoded_vgraph message'
            return response

        # 3. Serialize to VGH format
        vgh_text = self._serialize_vgh(self._graph_msg)
        response.success = True
        response.message = vgh_text
        return response

    @staticmethod
    def _serialize_vgh(graph_msg):
        """Convert a Graph ROS message to VGH text (same format as decoder)."""
        lines = []
        for node in graph_msg.nodes:
            parts = []
            parts.append(str(node.id))
            parts.append(str(node.freetype))
            # position
            parts.append(str(node.position.x))
            parts.append(str(node.position.y))
            parts.append(str(node.position.z))
            # surface dirs (2 vectors)
            if len(node.surface_dirs) >= 2:
                parts.append(str(node.surface_dirs[0].x))
                parts.append(str(node.surface_dirs[0].y))
                parts.append(str(node.surface_dirs[0].z))
                parts.append(str(node.surface_dirs[1].x))
                parts.append(str(node.surface_dirs[1].y))
                parts.append(str(node.surface_dirs[1].z))
            else:
                parts.extend(['0.0'] * 6)
            # flags
            parts.append(str(int(node.is_covered)))
            parts.append(str(int(node.is_frontier)))
            parts.append(str(int(node.is_navpoint)))
            parts.append(str(int(node.is_boundary)))
            # connect_nodes
            for c in node.connect_nodes:
                parts.append(str(c))
            parts.append('|')
            # poly_connects
            for c in node.poly_connects:
                parts.append(str(c))
            parts.append('|')
            # contour_connects
            for c in node.contour_connects:
                parts.append(str(c))
            parts.append('|')
            # trajectory_connects
            for c in node.trajectory_connects:
                parts.append(str(c))
            lines.append(' '.join(parts))
        return '\n'.join(lines)

    # ------------------------------------------------------------------
    # Graph upload (browser → robot)
    # ------------------------------------------------------------------

    def upload_graph_cb(self, msg):
        """Receive VGH text content, write temp file, publish path, cleanup."""
        vgh_content = msg.data
        if not vgh_content.strip():
            self.get_logger().warn('Empty VGH content received, ignoring.')
            return

        # Write to temp file
        fd, tmp_path = tempfile.mkstemp(suffix='.vgh', prefix='web_gui_')
        try:
            with os.fdopen(fd, 'w') as f:
                f.write(vgh_content)
            self.get_logger().info(f'VGH uploaded, temp file: {tmp_path}')

            # Publish path so decoder reads it
            path_msg = String()
            path_msg.data = tmp_path
            self.read_file_pub.publish(path_msg)

            # Give decoder time to read, then cleanup
            time.sleep(1.0)
        finally:
            try:
                os.unlink(tmp_path)
                self.get_logger().info(f'Temp file removed: {tmp_path}')
            except OSError:
                pass

    # ------------------------------------------------------------------
    # HTTP static file server
    # ------------------------------------------------------------------

    def _start_http_server(self):
        handler = partial(SimpleHTTPRequestHandler, directory=self.web_dir)
        self._http_server = HTTPServer(('0.0.0.0', self.http_port), handler)
        thread = threading.Thread(
            target=self._http_server.serve_forever, daemon=True)
        thread.start()
        self.get_logger().info(
            f'HTTP server started on http://0.0.0.0:{self.http_port}')


def main(args=None):
    rclpy.init(args=args)
    node = GraphFileBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
