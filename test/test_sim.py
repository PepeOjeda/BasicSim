import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
import xacro

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("worldFile", default_value="resources/test.yaml"),
    ]
#==========================

def launch_setup(context, *args, **kwargs):
    test_dir = os.path.join(get_package_share_directory("basic_sim"), "test")

    # robot description for state_publisher
    robot_desc1 = xacro.process_file(
        os.path.join(test_dir, "resources", "giraff.xacro"),
        mappings={"frame_ns": "giraff1"},
    )
    robot_desc1 = robot_desc1.toprettyxml(indent="  ")

    robot_desc2 = xacro.process_file(
        os.path.join(test_dir, "resources", "giraff.xacro"),
        mappings={"frame_ns": "giraff2"},
    )
    robot_desc2 = robot_desc2.toprettyxml(indent="  ")

    visualization_node1 = GroupAction(
        [
            PushRosNamespace("giraff1"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"use_sim_time": True, "robot_description": robot_desc1}],
            )
        ]
    )
    visualization_node2 = GroupAction(
        [
            PushRosNamespace("giraff2"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"use_sim_time": True, "robot_description": robot_desc2}],
            )
        ]
    )

    basic_sim = Node(
            package="basic_sim",
            executable="basic_sim",
            name="basic_sim",
            prefix = "xterm -e",
            parameters=[
                {"deltaTime": 0.1},
                {"speed": 10.0},
                {"worldFile": os.path.join(test_dir, LaunchConfiguration("worldFile").perform(context))}
                ],
        )
    
    keyboard_control1 = Node(
            package="keyboard_control",
            executable="keyboard_control_plus",
            prefix = "xterm -e",
            parameters=[
                {"linear_v_inc": 0.1},
                {"angular_v_inc": 0.1},
                {"publish_topic": "/giraff1/cmd_vel"}
                ],
        )
    keyboard_control2 = Node(
            package="keyboard_control",
            executable="keyboard_control_plus",
            prefix = "xterm -e",
            parameters=[
                {"linear_v_inc": 0.1},
                {"angular_v_inc": 0.1},
                {"publish_topic": "/giraff2/cmd_vel"}
                ],
        )
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=[
			"-d" + os.path.join(test_dir, "resources", "basic_sim.rviz")
		],
    )
    return [
        basic_sim,
        rviz,
        visualization_node1,
        visualization_node2,
        keyboard_control1,
        keyboard_control2
        ]


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
    ]
    
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    
    return  LaunchDescription(launch_description)