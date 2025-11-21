import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)


def generate_launch_description():
    startup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("kuka_iiqka_eac_driver"),
                "launch",
                "startup.launch.py",
            )
        )
    )

    moveit_config = (
        MoveItConfigsBuilder("kuka_lbr_iisy")
        .robot_description(
            os.path.join(
                get_package_share_directory("kuka_lbr_iisy_support"),
                "urdf",
                "lbr_iisy3_r760.urdf.xacro",
            ),
            {"mode": "mock"},
        )
        .robot_description_semantic(
            os.path.join(
                get_package_share_directory("kuka_lbr_iisy_moveit_config"),
                "urdf",
                "lbr_iisy3_r760.srdf",
            )   
        )
        .robot_description_kinematics(
            os.path.join(
                get_package_share_directory("kuka_lbr_iisy_moveit_config"),
                "config",
                "kinematics.yaml",
            )   
        )
        .trajectory_execution(
            os.path.join(
                get_package_share_directory("kuka_lbr_iisy_moveit_config"),
                "config",
                "moveit_controllers.yaml",
            )   
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .joint_limits(
            os.path.join(
                get_package_share_directory("kuka_lbr_iisy_support"),
                "config",
                "lbr_iisy3_r760_joint_limits.yaml",
            )
        )
        .to_moveit_configs()
    )

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"publish_planning_scene_hz": 30.0}],
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
                "planning_6_axis.rviz",
            ),
        ],
        parameters=[
            {
                "robot_description_kinematics": {
                    "manipulator": {
                        "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"
                    }
                }
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="mock"),
            DeclareLaunchArgument("use_gpio", default_value="false"),
            DeclareLaunchArgument("robot_family", default_value="lbr_iisy"),
            DeclareLaunchArgument("robot_model", default_value="lbr_iisy3_r760"),
            startup_launch,
            move_group_server,
            rviz,
        ]
    )
