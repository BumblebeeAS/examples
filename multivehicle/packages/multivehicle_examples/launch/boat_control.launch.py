import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("multivehicle_examples")
    control_params = os.path.join(pkg, "config", "blueboat_control.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_controller = LaunchConfiguration("use_controller")
    use_mission = LaunchConfiguration("use_mission")
    spawn = LaunchConfiguration("spawn")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_controller", default_value="true"),
        DeclareLaunchArgument("use_mission", default_value="false"),
        DeclareLaunchArgument("spawn", default_value="false"),
    ]

    spawn_boat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("multivehicle_sim"), "launch", "boat.launch.py"]
            )
        ),
        condition=IfCondition(spawn),
    )

    mixer = Node(
        package="multivehicle_examples",
        executable="blueboat_thrust_mixer.py",
        name="blueboat_thrust_mixer",
        output="screen",
        parameters=[control_params, {"use_sim_time": use_sim_time}],
    )

    odom_tf = Node(
        package="multivehicle_interface",
        executable="blueboat_odom_to_tf",
        name="blueboat_odom_to_tf",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "frame_id": "blueboat/odom",
                "child_frame_id": "blueboat/base_link",
            }
        ],
    )

    controller = Node(
        package="multivehicle_examples",
        executable="blueboat_waypoint_controller.py",
        name="blueboat_waypoint_controller",
        output="screen",
        parameters=[control_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_controller),
    )

    mission = Node(
        package="multivehicle_examples",
        executable="blueboat_mission.py",
        name="blueboat_square_mission",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_mission),
    )

    return LaunchDescription(args + [spawn_boat, mixer, odom_tf, controller, mission])
