from typing import Awaitable

from rclpy.node import Node

class LazyClient:
    node: Node
    topic: str
    service_type: type

    def __init__(self, node: Node, service_type: type, topic: str):
        self.node = node
        self.topic = topic
        self.service_type = service_type

    def call_async(self) -> Awaitable | None:
        client = self.node.create_client(self.service_type, self.topic)
        self.node.get_logger().info(f"Started call for {self.topic}...")

        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f"Service {self.topic} not available")
            return None

        request = self.service_type.Request()
        return client.call_async(request)
