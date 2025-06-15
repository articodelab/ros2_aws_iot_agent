import os
import rclpy
import json
from rclpy.node import Node
from uuid import uuid4
from ros2_aws_iot_agent.fleet_provisioner import (
    FleetProvisioner,
    FleetProvisioningConfig,
    FleetProvisioningResult,
)


class FleetProvisionerNode(Node):
    def __init__(self):
        super().__init__('fleet_provisioner_node')

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        node_id = uuid4().hex[:8]

        # Declare parameters with defaults
        self.declare_parameter("thing_name", f"roszen-{node_id}")
        self.declare_parameter("serial_number", f"roszen-{node_id}")
        self.declare_parameter("endpoint", "a1m0q9qwr15z0y-ats.iot.ap-northeast-2.amazonaws.com")
        self.declare_parameter("claim_cert", "src/ros2_aws_iot_agent/resource/certs/claim-certificate.pem.crt")
        self.declare_parameter("claim_key", "src/ros2_aws_iot_agent/resource/certs/claim-private.pem.key")
        self.declare_parameter("rootca", "src/ros2_aws_iot_agent/resource/certs/AmazonRootCA1.pem")
        self.declare_parameter("csr", "")
        self.declare_parameter("template_name", "roszen-device-provisioning-tempate")
        self.declare_parameter("template_parameters", "~/ros2_ws/config/template_paramters.json")
        self.declare_parameter("proxy_host", "")
        self.declare_parameter("proxy_port", 0)

        # Load parameters
        thing_name = self.get_parameter("thing_name").get_parameter_value().string_value
        serial_number = self.get_parameter("serial_number").get_parameter_value().string_value
        endpoint = self.get_parameter("endpoint").get_parameter_value().string_value
        claim_cert = os.path.expanduser(self.get_parameter("claim_cert").get_parameter_value().string_value)
        claim_key = os.path.expanduser(self.get_parameter("claim_key").get_parameter_value().string_value)
        rootca = os.path.expanduser(self.get_parameter("rootca").get_parameter_value().string_value)
        csr = self.get_parameter("csr").get_parameter_value().string_value
        csr = csr if csr else None
        template_name = self.get_parameter("template_name").get_parameter_value().string_value
        proxy_host = self.get_parameter("proxy_host").get_parameter_value().string_value
        proxy_host = proxy_host if proxy_host else None
        proxy_port = self.get_parameter("proxy_port").get_parameter_value().integer_value
        proxy_port = proxy_port if proxy_port > 0 else None

        config = FleetProvisioningConfig(
            thing_name=thing_name,
            endpoint=endpoint,
            port=8883,
            proxy_host=proxy_host,
            proxy_port=proxy_port,
            claim_cert=claim_cert,
            claim_key=claim_key,
            ca=rootca,
            csr=csr,
            template_name=template_name,
            template_parameters={
                "ThingName": thing_name,
                "SerialNumber": serial_number,
            },
            mqtt_version=5
        )

        # Provision the device
        self.get_logger().info(f"Provisioning Thing '{thing_name}' with template '{template_name}'")
        try:
            with FleetProvisioner(config) as provisioner:
                result: FleetProvisioningResult = provisioner.provision()
            self.get_logger().info(f"Provisioned successfully: Thing={result.thing_name}")
            self.save_credentials(result)

        except Exception as e:
            self.get_logger().error(f"Provisioning failed: {e}")


    def timer_callback(self):
        self.get_logger().info('Hello World: %d' % self.i)
        self.i += 1


    def save_credentials(self, result: FleetProvisioningResult):
        os.makedirs("src/ros2_aws_iot_agent/resource/certs", exist_ok=True)

        cert_file = f"src/ros2_aws_iot_agent/resource/certs/{result.thing_name}-certificate.pem.crt"
        key_file = f"src/ros2_aws_iot_agent/resource/certs/{result.thing_name}-private.pem.key"
        info_file = f"src/ros2_aws_iot_agent/resource/certs/{result.thing_name}-register.json"

        with open(cert_file, "w") as f:
            f.write(result.certificate_pem)

        if result.private_key:
            with open(key_file, "w") as f:
                f.write(result.private_key)

        with open(info_file, "w") as f:
            json.dump({
                "thing_name": result.thing_name,
                "certificate_id": result.certificate_id
            }, f, indent=2)

        self.get_logger().info(f"Saved certificate and registration info for {result.thing_name}")


def main(args=None):
    rclpy.init(args=args)
    node = FleetProvisionerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
