from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro
from launch.event_handlers import OnProcessExit


import os


def generate_launch_description():
    # Convert the xacro file to URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("bolt_description"),
                    "urdf",
                    "bolt.urdf.xacro",
                ]
            ),
            " ",
            "mujoco_model:=",
            PathJoinSubstitution(
                [
                    FindPackageShare("bolt_mujoco_simulation"),
                    "etc",
                    "world.xml",
                ]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("bolt_mujoco_simulation"),
            "config",
            "controller_manager.yaml",
        ]
    )

    gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        prefix="gdbserver localhost:3000",
        emulate_tty=True,
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    return LaunchDescription(
        [
            rsp,
            control_node,
            joint_state_broadcaster_spawner,
            # gui,
            # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        ]
    )
