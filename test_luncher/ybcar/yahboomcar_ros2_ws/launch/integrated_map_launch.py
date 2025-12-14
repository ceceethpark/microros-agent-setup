from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def _include_launch(context, *args, **kwargs):
    # Resolve package and launch names from LaunchConfiguration
    elements = []
    for key in ('bringup_pkg', 'bringup_launch', 'imu_pkg', 'imu_launch', 'gmapping_pkg', 'gmapping_launch'):
        elements.append(LaunchConfiguration(key).perform(context))

    bringup_pkg, bringup_launch, imu_pkg, imu_launch, gmapping_pkg, gmapping_launch = elements

    actions = []
    try:
        bringup_path = os.path.join(get_package_share_directory(bringup_pkg), 'launch', bringup_launch)
        actions.append( ('bringup', bringup_path) )
    except Exception:
        pass

    try:
        imu_path = os.path.join(get_package_share_directory(imu_pkg), 'launch', imu_launch)
        actions.append( ('imu', imu_path) )
    except Exception:
        pass

    try:
        gmapping_path = os.path.join(get_package_share_directory(gmapping_pkg), 'launch', gmapping_launch)
        actions.append( ('gmapping', gmapping_path) )
    except Exception:
        pass

    # Execute each launch file as a subprocess via ros2 launch (simple and robust)
    from launch.actions import ExecuteProcess
    for name, path in actions:
        actions.append( ExecuteProcess(cmd=['ros2','launch', path], output='screen') )

    return actions


def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments to allow caller to override package/launch names
    ld.add_action(DeclareLaunchArgument('bringup_pkg', default_value='robot_bringup'))
    ld.add_action(DeclareLaunchArgument('bringup_launch', default_value='bringup.launch.py'))
    ld.add_action(DeclareLaunchArgument('imu_pkg', default_value='imu_ws'))
    ld.add_action(DeclareLaunchArgument('imu_launch', default_value=os.path.join('launch','ekf_launch.py')))
    ld.add_action(DeclareLaunchArgument('gmapping_pkg', default_value='yahboomcar_nav'))
    ld.add_action(DeclareLaunchArgument('gmapping_launch', default_value='map_gmapping_launch.py'))

    # OpaqueFunction will resolve the LaunchConfigurations and start each launch via ros2 launch subprocess
    ld.add_action(OpaqueFunction(function=_include_launch))

    return ld
