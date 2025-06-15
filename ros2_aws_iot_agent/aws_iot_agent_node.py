import os
import rclpy
import subprocess
from rclpy.node import Node

CERT_DIR = os.path.expanduser("~/ros2_ws/src/ros2_aws_iot_agent/resource/certs")

class AwsIotAgentNode(Node):
    def __init__(self):
        super().__init__('aws_iot_agent_node')
        self.timer = self.create_timer(1.0, self.check_and_launch)

    def check_and_launch(self):
        self.timer.cancel()
        has_cert = any(f.endswith("-register.json") for f in os.listdir(CERT_DIR))
        if has_cert:
            subprocess.Popen(["ros2", "run", "ros2_aws_iot_agent", "device_shadow_node"])
        else:
            subprocess.run(["ros2", "run", "ros2_aws_iot_agent", "fleet_provisioner_node"], check=True)
            subprocess.Popen(["ros2", "run", "ros2_aws_iot_agent", "device_shadow_node"])


def main(args=None):
    rclpy.init(args=args)
    node = AwsIotAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()