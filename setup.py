from setuptools import find_packages, setup

package_name = 'ros2_aws_iot_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gomma',
    maintainer_email='dshwang@articodelab.com',
    description='ROS 2 nodes for AWS IoT Fleet Provisioning and Shadow',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aws_iot_agent_node = ros2_aws_iot_agent.aws_iot_agent_node:main',
            'fleet_provisioner_node = ros2_aws_iot_agent.fleet_provisioner_node:main',
            'device_shadow_node = ros2_aws_iot_agent.device_shadow_node:main',
        ],
    },
)
