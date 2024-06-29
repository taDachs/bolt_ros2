#!/usr/bin/env python3
import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # get the package share directory
    bolt_description_dir = get_package_share_directory("bolt_description")
    description_file = os.path.join(bolt_description_dir, "urdf", "bolt.urdf.xacro")

    # Convert the xacro file to URDF
    doc = xacro.process_file(description_file)
    robot_desc = doc.toprettyxml(indent="  ")
    params = {"robot_description": robot_desc}
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[params],
    )

    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    return LaunchDescription([rsp, jsp])
