from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform from base_link to base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        # Diff Drive controller node (if it's not launched from Gazebo or another system)
        Node(
            package='send_vel',  # Change this if needed
            executable='diff_driver',
            name='diff_driver'
        ),

        # Straight path publisher: listens to 2D Goal from RViz
        Node(
            package='victor_planning',
            executable='rviz_goal_curve_path_publisher',
            name='rviz_goal_curve_path_publisher'
        ),

        # Pure Pursuit controller
        Node(
            package='victor_motion',
            executable='pure_pursuit',
            name='pure_pursuit_motion_planner_node',
            remappings=[
                ('/cmd_vel', '/diff_cont/cmd_vel_unstamped')  # if your diff_drive_controller uses this
            ],
            parameters=[
                {'look_ahead_distance': 0.5},
                {'max_linear_velocity': 0.3},
                {'max_angular_velocity': 1.0}
            ]
        )
    ])
