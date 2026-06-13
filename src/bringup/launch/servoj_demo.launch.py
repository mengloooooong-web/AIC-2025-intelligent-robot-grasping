from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare a single launch argument for the entire pose array
    pose_arg = DeclareLaunchArgument(
        'pose',
        default_value='[0.001, 0.0, 0.0, 0.0, 0.0, 0.001]',
        description='Pose values as a list of 6 doubles'
    )

    # Create a Node with the pose parameter
    node = Node(
        package='jaka_driver',
        executable='servoj_demo',
        name='servoj_demo',
        output='screen',
        parameters=[{
            'pose': LaunchConfiguration('pose')
        }]
    )

    return LaunchDescription([pose_arg, node])