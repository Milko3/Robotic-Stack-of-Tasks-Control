"""Run our Stack of Tasks controller on the Panda (ros2_control mock hardware) and show it in RViz.

    ros2 launch sot_ros sot.launch.py              # with RViz
    ros2 launch sot_ros sot.launch.py rviz:=false  # without a display
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context):
    share = get_package_share_directory("sot_ros")
    robot_description = xacro.process_file(
        os.path.join(share, "urdf", "panda_sot.urdf.xacro")).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},
                    os.path.join(share, "config", "controllers.yaml")],
        output="screen",
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    sot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sot_controller", "--controller-manager", "/controller_manager"],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(share, "rviz", "sot.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="log",
    )
    # Start the SoT controller only once joint states are being broadcast.
    start_sot_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[sot_controller]))

    return [robot_state_publisher, control_node, joint_state_broadcaster, start_sot_after_jsb, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
        OpaqueFunction(function=make_nodes),
    ])
