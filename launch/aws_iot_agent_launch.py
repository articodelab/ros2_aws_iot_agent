from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_aws_iot_agent',
            executable='aws_iot_agent_node',
            name='taler'
        )
        ],
#        prefix=["gdbserver :3000"],
    )


def main():
    ls = launch.LaunchService()
    ld = generate_launch_description()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()