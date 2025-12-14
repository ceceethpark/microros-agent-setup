from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    # Simple scaffold: runs your bringup launch via ros2 launch
    # Edit the package/launch name if your bringup differs.
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'launch', 'robot_bringup', 'bringup.launch.py'],
        output='screen'
    ))

    return ld
