
# ros2_aws_iot_fleet_agent

An open-source ROS 2 agent that integrates ROS 2-based devices and robots with AWS IoT for remote management and monitoring.  
By leveraging AWS IoT Core’s Fleet Provisioning, Device Shadow, and Remote Management features, this agent enables efficient remote monitoring and maintenance of ROS 2 devices and distributed robotic systems.

---

## 🔥 Key Features

- AWS IoT Fleet Provisioning (Automatic Thing registration)
- AWS IoT Device Shadow integration (State reporting and remote control)
- Modular ROS 2 Node architecture
- SaaS-ready fleet management system integration
- Future support for AWS IoT Jobs and OTA updates

---

## 🚀 Installation & Setup Guide

### 1. ROS 2 (Humble) Environment Setup

This package is based on ROS 2 Humble (Recommended OS: Ubuntu 22.04).

```bash
source /opt/ros/humble/setup.bash
```

---

### 2. Create Workspace and Clone Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/articodelab/ros2_aws_iot_agent.git
```

---

### 3. Install Dependencies and Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

### 4. AWS IoT Core Preparation

Before integration, prepare AWS IoT Core as follows:

- Enable AWS IoT Core service
- Create Fleet Provisioning templates
- Generate Claim Certificate and Private Key
- Download Amazon Root CA
- Configure AWS IoT Policies and Roles

*Detailed AWS setup documentation will be provided separately.*

---

### 5. Environment Variable Configuration

Set the following environment variables for authentication and device identity:

| Variable | Description |
|------------|-------------|
| `AWS_ENDPOINT` | AWS IoT Core endpoint |
| `CLAIM_CERT` | Path to the claim certificate |
| `CLAIM_KEY` | Path to the claim private key |
| `ROOT_CA` | Path to AmazonRootCA1 |
| `THING_NAME` | Thing Name (optional) |
| `SERIAL_NUMBER` | Device serial number (optional) |

Example:

```bash
export AWS_ENDPOINT="xxxxxxxxxxxxxx-ats.iot.ap-northeast-2.amazonaws.com"
export CLAIM_CERT="~/certs/claim-certificate.pem.crt"
export CLAIM_KEY="~/certs/claim-private.pem.key"
export ROOT_CA="~/certs/AmazonRootCA1.pem"
export THING_NAME="ros2-fleet-device-001"
export SERIAL_NUMBER="device-001"
```

---

### 6. Execution Examples

**Run Fleet Provisioning Node**

```bash
ros2 launch ros2_aws_iot_fleet_agent_pkg fleet_provisioner.launch.py
```

**Run Device Shadow Node**

```bash
ros2 launch ros2_aws_iot_fleet_agent_pkg device_shadow.launch.py
```

*You can also pass parameters via the launch file.*

---

## 🗺 Architecture Overview

```plaintext
+-----------------------------------------------------------+
|             SaaS Operation Management System              |
|          (Web Dashboard, Analytics, OTA Server)           |
+------------------------▲----------------------------------+
                         │
               AWS IoT Core (Cloud Side)
     +------------+-----------+------------+--------------+
     | Device Shadow | Fleet Provisioning | IoT Jobs ... |
     +------------+-----------+------------+--------------+
                         │
            +------------▼------------+
            |   ros2_aws_iot_fleet_agent (Edge Device)  |
            +--------------------------+
            | FleetProvisionerNode     |
            | DeviceShadowNode         |
            +--------------------------+
            | ROS 2 Application Layer  |
            +--------------------------+
```

---

## 📌 Future Roadmap

- AWS IoT Jobs integration (Remote command & deployment management)
- OTA update examples
- SaaS platform SDK and API integration
- WebUI-based real-time monitoring dashboard
- Remote fault analysis and cloud logging integration
- Multi-device fleet management support
- Enhanced security (X.509 / PKI-based authentication)

---

## 🤝 Contribution

- Pull Requests are welcome
- Open issues for discussions and improvements
- Collaboration on SaaS platform integration is highly encouraged

---

## 📄 License

This project is licensed under Apache License 2.0.

---

## 🙋 Contact

- Company: articodelab
- Maintainer: Arti
- Email: dshwang@articodelab.com

---
