from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
import os

def generate_launch_description():
    return LaunchDescription(
        [
            # Global arguments declarations
            DeclareLaunchArgument("offline", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("colored_pointcloud", default_value="true"),
            DeclareLaunchArgument("visualize_segmented_scene", default_value="true"),
            DeclareLaunchArgument(
                "semantic_scene_segmenter",
                default_value="yoso",
                description="The method to segment the semantic scene (if off, the baseline)",
                choices=["yoso","yoso_amp", "pfcn", "off"],
            ),
            DeclareLaunchArgument(
                "use_isaac_ros",
                default_value="false",
                description="Use isaac_ros_depth_image_proc for GPU acceleration (Jetson only)",
            ),
            DeclareLaunchArgument(
                "use_sync_converter",
                default_value="false",
                description="Use sync converter for synchronizing topics (required for isaac_ros_depth_image_proc)",
            ),
            # Topics
            DeclareLaunchArgument("camera_frame", default_value="camera"),
            DeclareLaunchArgument("sensor_config", default_value="SMapper_RealSense"),
            DeclareLaunchArgument(
                "rgb_image_topic", default_value="/camera/realsense/color/image_raw"
            ),
            DeclareLaunchArgument(
                "rgb_camera_info_topic",
                default_value="/camera/realsense/color/camera_info",
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/realsense/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/camera/depth/points",
            ),
            # VS-Graphs Node
            Node(
                name="vs_graphs",
                package="vs_graphs",
                executable="ros_rgbd",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("offline")},
                    {
                        "voc_file": LaunchConfiguration(
                            "voc_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/Vocabulary/ORBvoc.txt.bin",
                            ],
                        )
                    },
                    {
                        "settings_file": LaunchConfiguration(
                            "settings_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/config/RGB-D/",
                                LaunchConfiguration("sensor_config"),
                                ".yaml",
                            ],
                        )
                    },
                    {
                        "sys_params_file": LaunchConfiguration(
                            "sys_params_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/config/system_params.yaml",
                            ],
                        )
                    },
                    {"roll": 0.0},
                    {"yaw": 1.5697},
                    {"pitch": -1.5697},
                    {"frame_map": "map"},
                    {"frame_world": "world"},
                    {"frame_camera": "camera"},
                    {"enable_pangolin": False},
                    {"static_transform": True},
                    {"colored_pointcloud": False},
                    {"publish_pointclouds": True},
                ],
                remappings=[
                    ("/camera/rgb/image_raw", LaunchConfiguration("rgb_image_topic")),
                    ("/camera/depth_registered/image_raw",LaunchConfiguration("depth_image_topic"),),
                    ("/camera/depth/points", LaunchConfiguration("pointcloud_topic")),
                ],
            ),
            # Static Transforms
            Node(
                package="tf2_ros",
                name="map_to_map_elevated",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "map_elevated"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="bc_to_se",
                arguments=["0", "-3", "0", "0", "0", "0", "build_comp", "struc_elem"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_bc",
                arguments=["0", "0.5", "0", "0", "0", "0", "world", "build_comp"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_to_camera_optical",
                arguments=[
                    "0", "0", "0", "0", "0", "0",
                    "camera", "d400_color",
                ],
            ),
            # RViz
            Node(
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=[
                    "-d",
                    [
                        get_package_share_directory("vs_graphs"),
                        "/config/Visualization/vsgraphs_rgbd.rviz",
                    ],
                ],
                output="screen",
            ),
            # Standard depth_image_proc (CPU)
            ComposableNodeContainer(
                condition=UnlessCondition(LaunchConfiguration("use_isaac_ros")),
                name="depth_image_proc_container",
                package="rclcpp_components",
                namespace="",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyzrgbNode",
                        name="point_cloud_xyzrgb_node",
                        remappings=[
                            ("rgb/camera_info", LaunchConfiguration("rgb_camera_info_topic")),
                            ("rgb/image_rect_color", LaunchConfiguration("rgb_image_topic")),
                            ("depth_registered/image_rect", LaunchConfiguration("depth_image_topic")),
                            ("points", "/camera/depth/points"),
                        ],
                    ),
                ],
            ),
            # Isaac ROS depth_image_proc (GPU - Jetson only)(only rgb images are accepted, topics must be synchronized)
            ComposableNodeContainer(
                condition=IfCondition(LaunchConfiguration("use_isaac_ros")),
                name="depth_image_proc_container",
                package="rclcpp_components",
                namespace="",
                executable="component_container",
                prefix=[get_package_share_directory("vs_graphs"), "/launch/jetson/component_container_isaac.sh"],
                composable_node_descriptions=[
                    ComposableNode(
                        package="isaac_ros_depth_image_proc",
                        plugin="nvidia::isaac_ros::depth_image_proc::PointCloudXyzrgbNode",
                        name="point_cloud_xyzrgb_node",
                        remappings=[
                            ("rgb/camera_info", "/camera/rgb/camera_info_sync"),
                            ("rgb/image_rect_color", "/camera/rgb/image_sync"),
                            ("depth_registered/image_rect", "/camera/depth/image_sync"),
                            ("points", "/camera/depth/points"),
                        ],

                    ),
                ],
            ),
            # Sync Converter Node (only for isaac_ros_depth_image_proc, to synchronize topics if needed)
            Node(
                condition=IfCondition(LaunchConfiguration("use_sync_converter")),
                name="sync_converter",
                package="vs_graphs",
                executable="sync_converter.py",
                output="screen"
            ),
            # PointCloud fix node (only for Jetson with isaac_ros_depth_image_proc, as it outputs unorganized pointclouds)
            Node(
                condition=IfCondition(LaunchConfiguration("use_isaac_ros")),
                name="fix_pointcloud",
                package="vs_graphs",
                executable="fix_pointcloud.py",
                output="screen"
            ),
            # Semantic Scene Segmenter Node
            Node(
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration("semantic_scene_segmenter"), "yoso"
                    )
                ),
                name="segmenter_ros",
                package="segmenter_ros",
                executable="segmenter_yoso.py",
                output="screen",
                parameters=[
                    {"visualize": LaunchConfiguration("visualize_segmented_scene")}
                ],
                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_yoso.yaml",
                    ],
                ],
            ),
            Node(
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration("semantic_scene_segmenter"), "yoso"
                    )
                ),
                name="segmenter_ros",
                package="segmenter_ros",
                executable="segmenter_yoso.py",
                output="screen",
                parameters=[
                    {"visualize": LaunchConfiguration("visualize_segmented_scene")},
                    {"enable_amp": True},
                    
                ],
                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_yoso.yaml",
                    ],
                ],
            ),
            Node(
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration("semantic_scene_segmenter"), "pfcn"
                    )
                ),
                name="segmenter_ros",
                package="segmenter_ros",
                executable="segmenter_pFCN.py",
                output="screen",
                parameters=[
                    {"visualize": LaunchConfiguration("visualize_segmented_scene")}
                ],
                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_pFCN.yaml",
                    ],
                ],
            ),
        ]
    )