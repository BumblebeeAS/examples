from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}

    return LaunchDescription(
        [
            Node(
                package="multivehicle_interface",
                executable="ground_truth_to_mavros",
                name="ground_truth_to_mavros",
                output="screen",
                parameters=[use_sim_time],
            ),
            Node(
                package="bluerov_tasks",
                executable="bluerov_movement.py",
                name="bluerov_movement",
                output="screen",
                parameters=[use_sim_time],
            ),
        ]
    )
