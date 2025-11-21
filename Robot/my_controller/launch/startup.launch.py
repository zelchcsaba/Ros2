import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode, Node

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                os.path.join(
                    get_package_share_directory("kuka_lbr_iisy_support"),
                    "urdf",
                    "lbr_iisy3_r760.urdf.xacro",
                ),
                " mode:=mock",
            ],
            on_stderr="capture",
        )
    }

    control_node = Node(
        package="kuka_drivers_core",
        executable="control_node",
        parameters=[
            robot_description,
            os.path.join(
                get_package_share_directory("my_controller"),
                "config",
                "ros2_controller_config.yaml",
            ),
            {"hardware_components_initial_state": {"unconfigured": ["lbr_iisy3_r760"]}},
        ],
    )

    robot_manager_node = LifecycleNode(
        name=["robot_manager"],
        namespace="",
        package="kuka_rsi_driver",
        executable=("robot_manager_node_rsi_only"),
        parameters=[
            os.path.join(
                get_package_share_directory("kuka_rsi_driver"),
                "config",
                "driver_config.yaml",
            ),
            {
                "robot_model": "lbr_iisy3_r760",
                "position_controller_name": "my_controller",
            },
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--inactive",
        ],
    )

    my_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "my_controller",
            "-c",
            "/controller_manager",
            "--param-file",
            os.path.join(
                get_package_share_directory("my_controller"),
                "config",
                "my_controller.yaml",
            ),
            "--inactive",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("kuka_resources"),
                "config",
                "view_6_axis_urdf.rviz",
            ),
        ],
    )

    return LaunchDescription(
        [
            control_node,
            robot_manager_node,
            robot_state_publisher,
            joint_state_broadcaster,
            my_controller,
            rviz,
        ]
    )
