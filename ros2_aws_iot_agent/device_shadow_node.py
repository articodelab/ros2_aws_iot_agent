import os
import json
import threading
import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros2_aws_iot_agent.shadow_client import (
  ShadowClient,
  ShadowClientConfig
)

class DeviceShadowNode(Node):
    def __init__(self):
        super().__init__('device_shadow_node')

        self.subscription = self.create_subscription(
            String, "sensehat/sensor", self.listener_callback, 10
        )

        # AWS IoT device shadow
        self.declare_parameter("thing_name", "")
        self.declare_parameter("endpoint", "a1m0q9qwr15z0y-ats.iot.ap-northeast-2.amazonaws.com")
        self.declare_parameter("cert", "src/ros2_aws_iot_agent/resource/certs/roszen-certificate.pem.crt")
        self.declare_parameter("key", "src/ros2_aws_iot_agent/resource/certs/roszen-private.pem.key")
        self.declare_parameter("ca", "src/ros2_aws_iot_agent/resource/certs/AmazonRootCA1.pem")

        thing_name = self.get_parameter("thing_name").get_parameter_value().string_value
        endpoint = self.get_parameter("endpoint").get_parameter_value().string_value
        cert = os.path.expanduser(self.get_parameter("cert").get_parameter_value().string_value)
        key = os.path.expanduser(self.get_parameter("key").get_parameter_value().string_value)
        ca = os.path.expanduser(self.get_parameter("ca").get_parameter_value().string_value)

        config = ShadowClientConfig(
            thing_name=thing_name,
            endpoint=endpoint,
            cert=cert,
            key=key,
            ca=ca,
        )

        self.shadow_client = ShadowClient(config, {})

        # Create a background thread for running asyncio event loop
        self.loop_thread = threading.Thread(target=self._run_asyncio_loop)
        self.loop_thread.start()

    def _run_asyncio_loop(self):
        """Run the asyncio loop in a separate thread to avoid blocking ROS2 spin()"""
        asyncio.run(self.start_shadow())

    async def start_shadow(self):
        try:
            self.get_logger().info("Starting AWS IoT Device Shadow")
            await self.shadow_client.start()
        except Exception as e:
            self.get_logger().error(f"Provisioning failed: {e}")

    def listener_callback(self, msg):
        """ROS2 callback to handle incoming messages and update the shadow state"""
        self.get_logger().info(f"Received ROS2 message: {msg.data}")
        try:
            # Convert the received message data into a dictionary and update the shadow state
            new_state = json.loads(msg.data)
            self.shadow_client.update_device_state(new_state)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode message data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DeviceShadowNode()
    
    # Create a MultiThreadedExecutor to allow for concurrent running of ROS2 and asyncio tasks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
