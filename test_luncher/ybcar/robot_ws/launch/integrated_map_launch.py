from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    # Integrates bringup + imu EKF + gmapping as a simple scaffold.
    # Adjust package/launch names to match your workspace.

    # 1) robot bringup
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'launch', 'robot_bringup', 'bringup.launch.py'],
        output='screen'
    ))

    # 2) imu EKF (imu_ws)
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'launch', 'imu_ws', 'launch/ekf_launch.py'],
        output='screen'
    ))

    # 3) gmapping (yahboomcar_nav or gmapping package)
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'launch', 'yahboomcar_nav', 'map_gmapping_launch.py'],
        output='screen'
    ))

    return ld
