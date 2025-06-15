import os
import json
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

        register_file = "roszen-register.json"
        register_file_path = os.path.join(CERT_DIR, register_file)

        if not os.path.exists(register_file_path):
            self.get_logger().info("No register file found. Running FleetProvisioner.")
            subprocess.run([
                "ros2", "run", "ros2_aws_iot_agent", "fleet_provisioner_node"
            ], check=True)

        self.get_logger().info(f"Found register file: {register_file_path}")
        with open(register_file_path, 'r') as f:
            data = json.load(f)

        thing_name = data.get("thing_name")
        if not thing_name:
            self.get_logger().error("thing_name not found in register file.")
            return

        subprocess.Popen([
            "ros2", "run", "ros2_aws_iot_agent", "device_shadow_node",
            "--ros-args", "-p", f"thing_name:={thing_name}"
        ])
 

def main(args=None):
    rclpy.init(args=args)
    node = AwsIotAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()