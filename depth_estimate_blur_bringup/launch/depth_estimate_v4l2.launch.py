import os
import sys
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "video_device",
            default_value="/dev/video0",
            description="input video source"
        )
    ]
    container = ComposableNodeContainer(
                name='depth_estimate_blur_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='v4l2_camera',
                        plugin='v4l2_camera::V4L2Camera',
                        name='v4l2_camera',
                        parameters=[{
                            "video_device": LaunchConfiguration("video_device"),
                            "image_size": [640,480]
                        }]),
                    ComposableNode(
                        package='depth_estimate_blur_node',
                        plugin='depth_estimate_blur_node::DepthEstimateBlurNode',
                        name='depth_estimate_blur_node'
                    ),
                ],
                output='screen',
        )

    rqt = launch_ros.actions.Node(
        package="rqt_graph", executable="rqt_graph",
    )

    return launch.LaunchDescription(
        launch_args +
        [
            container,
            # rqt_graph,
        ]
    )