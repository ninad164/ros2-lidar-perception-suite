import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('pcl_car_segmentation')

    params_file = os.path.join(
        package_share,
        'config',
        'perception_pipeline.yaml'
    )

    rviz_config = os.path.join(
        package_share,
        'config',
        'car_segmentation_config.rviz'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/kitti/point_cloud',
        description='Input PointCloud2 topic'
    )

    input_topic = LaunchConfiguration('input_topic')

    cloud_pre_processing = Node(
        package='pcl_car_segmentation',
        executable='cloud_pre_processing',
        name='cloud_pre_processing',
        output='screen',
        parameters=[
            params_file,
            {'input_topic': input_topic}
        ]
    )

    car_segmentation = Node(
        package='pcl_car_segmentation',
        executable='car_segmentation_by_clusters',
        name='car_segmentation',
        output='screen',
        parameters=[params_file]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        input_topic_arg,
        cloud_pre_processing,
        car_segmentation,
        rviz
    ])