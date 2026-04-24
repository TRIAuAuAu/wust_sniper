import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ---------- Launch 参数 ----------
    use_mqtt_arg = DeclareLaunchArgument(
        'use_mqtt', default_value='false',
        description='Use MQTT for video streaming (true) or ROS2 (false)'
    )
    use_mqtt = LaunchConfiguration('use_mqtt')

    # ---------- 路径参数 ----------
    node_params = os.path.join(
        get_package_share_directory('bringup'), 'config', 'node_params.yaml')
    # 确定项目根目录，用于调试截图路径
    launch_path = Path(__file__).resolve()
    project_root = launch_path.parents[3]   # 适配源码/安装环境（按你的原逻辑）
    debug_dump_dir = str(project_root / 'sniper_debug_imgs')

    # 编码器参数（示例）
    encode_size = 300
    target_bitrate_kbytes = 10.0
    target_bitrate_kbps = int(target_bitrate_kbytes * 8)

    # ---------- 编码端容器（相机 + 编码器，始终启动） ----------
    encoder_container = ComposableNodeContainer(
        name='sniper_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='hik_camera',
                parameters=[node_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='doorlock_sniper',
                plugin='doorlock_sniper::VideoEncoderNode',
                name='video_encoder',
                parameters=[node_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )

    # ---------- 解码端（仅 ROS2 调试时启动） ----------
    decoder_container = ComposableNodeContainer(
        name='decoder_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='doorlock_decoder_cpp',
                plugin='doorlock_sniper::VideoDecoderNode',
                name='video_decoder',
                parameters=[node_params]
            )
        ],
        output='screen',
    )

    return LaunchDescription([
        use_mqtt_arg,
        encoder_container,
        decoder_container,
    ])