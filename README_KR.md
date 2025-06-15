
# ros2_aws_iot_fleet_agent

ROS 2 기반 디바이스 및 로봇의 AWS IoT 연동과 운영 관리를 지원하는 오픈소스 ROS 2 Agent입니다.  
AWS IoT Core의 Fleet Provisioning, Device Shadow, Remote Management 기능을 활용하여 다양한 ROS 2 기반 시스템의 원격 모니터링과 유지 관리를 손쉽게 구현할 수 있습니다.

---

## 🔥 주요 기능

- ✅ AWS IoT Fleet Provisioning (자동 Thing 등록)
- ✅ AWS IoT Device Shadow 연동 (상태 보고 및 원격 설정)
- ✅ ROS 2 Node 기반 모듈화
- ✅ SaaS 기반 운영관리 시스템과 연동 가능성 확보
- ✅ 향후 AWS IoT Jobs, OTA Update 확장 가능

---

## 🚀 설치 및 실행 가이드

### 1. ROS 2 (Humble) 환경 준비

본 패키지는 ROS 2 Humble 버전을 기준으로 개발되었습니다. (Ubuntu 22.04 권장)

```bash
source /opt/ros/humble/setup.bash
```

---

### 2. 작업 공간 생성 및 소스 코드 복제

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/articodelab/ros2_aws_iot_agent.git
```

---

### 3. 의존성 설치 및 빌드

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

### 4. AWS IoT Core 환경 준비

AWS IoT Core와 연동하기 위해 아래 사전 준비가 필요합니다:

- AWS IoT Core 서비스 활성화
- Fleet Provisioning template 작성
- Claim Certificate 및 Private Key 발급
- Amazon Root CA 다운로드
- AWS IoT 정책 및 Role 설정

※ AWS 설정 가이드는 추후 별도 문서로 상세 제공 예정입니다.

---

### 5. 환경 변수 설정

아래 환경 변수를 설정하여 인증 정보를 제공합니다:

| 환경 변수 | 설명 |
|------------|------|
| `AWS_ENDPOINT` | AWS IoT Core endpoint |
| `CLAIM_CERT` | Claim 인증서 경로 |
| `CLAIM_KEY` | Claim 개인키 경로 |
| `ROOT_CA` | AmazonRootCA1 경로 |
| `THING_NAME` | Thing Name (옵션) |
| `SERIAL_NUMBER` | 디바이스 고유 시리얼 넘버 (옵션) |

환경 변수 설정 예시:

```bash
export AWS_ENDPOINT="xxxxxxxxxxxxxx-ats.iot.ap-northeast-2.amazonaws.com"
export CLAIM_CERT="~/certs/claim-certificate.pem.crt"
export CLAIM_KEY="~/certs/claim-private.pem.key"
export ROOT_CA="~/certs/AmazonRootCA1.pem"
export THING_NAME="ros2-fleet-device-001"
export SERIAL_NUMBER="device-001"
```

---

### 6. 실행 예제

**Fleet Provisioning Node 실행**

```bash
ros2 launch ros2_aws_iot_fleet_agent_pkg fleet_provisioner.launch.py
```

**Device Shadow Node 실행**

```bash
ros2 launch ros2_aws_iot_fleet_agent_pkg device_shadow.launch.py
```

※ launch 파일 내부에서 ROS2 파라미터를 통해 개별 설정 가능

---

## 🗺 프로젝트 아키텍처 개념도

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
            | ROS 2 기반 Application   |
            +--------------------------+
```

---

## 📌 향후 확장 계획

- ✅ AWS IoT Jobs 연동 (원격 명령 및 배포 관리)
- ✅ OTA 업데이트 기능 예제 추가
- ✅ SaaS 운영관리 플랫폼 연동 SDK 및 API 제공
- ✅ WebUI 기반 실시간 모니터링 대시보드 제공
- ✅ 원격 장애 분석 및 로깅 시스템 연동
- ✅ 멀티 디바이스 Fleet 관리 확장
- ✅ 보안강화 (X.509, PKI 기반 인증 체계 고도화)

---

## 🤝 기여 방법

- Pull Request 환영
- Issue 등록 및 토론 참여
- 운영관리 SaaS 연동 개발 제안 가능

---

## 📄 라이선스

본 프로젝트는 Apache License 2.0을 따릅니다.

---

## 🙋 문의

- 회사: articodelab
- Maintainer: Arti
- Email: dshwang@articodelab.com

---
