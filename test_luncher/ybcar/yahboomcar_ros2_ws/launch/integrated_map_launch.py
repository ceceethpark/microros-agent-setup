from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def make_include(pkg, launch_file):
    pkg_share = get_package_share_directory(pkg)
    path = os.path.join(pkg_share, 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments so users can override package/launch names
    ld.add_action(DeclareLaunchArgument('bringup_pkg', default_value='robot_bringup'))
    ld.add_action(DeclareLaunchArgument('bringup_launch', default_value='bringup.launch.py'))
    ld.add_action(DeclareLaunchArgument('imu_pkg', default_value='imu_ws'))
    ld.add_action(DeclareLaunchArgument('imu_launch', default_value=os.path.join('launch','ekf_launch.py')))
    ld.add_action(DeclareLaunchArgument('gmapping_pkg', default_value='yahboomcar_nav'))
    ld.add_action(DeclareLaunchArgument('gmapping_launch', default_value='map_gmapping_launch.py'))

    # Attempt to include common defaults if present
    try:
        ld.add_action(make_include('robot_bringup', 'bringup.launch.py'))
    except Exception:
        pass

    try:
        ld.add_action(make_include('imu_ws', os.path.join('launch','ekf_launch.py')))
    except Exception:
        pass

    try:
        ld.add_action(make_include('yahboomcar_nav', 'map_gmapping_launch.py'))
    except Exception:
        pass

    return ld
