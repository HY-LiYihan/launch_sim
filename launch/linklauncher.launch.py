import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 摄像机光学坐标系变换 (camera_link -> camera_optical_frame)
        # 作用：连接机器人本体坐标系和视觉算法坐标系
        # 变换：位置重合 (0 0 0)，旋转 (Yaw=-90, Pitch=0, Roll=-90) 使得 Z轴向前, Y轴向下
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            # 参数格式: x y z yaw pitch roll frame_id child_frame_id
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_optical_frame']
        ),

        # 2. 发射器坐标系 (map -> launcher_link)
        # 作用：模拟一个架设在地上，枪口抬高 10 度的发射器
        # 变换：假设高度 1.0m，Pitch = -10度 (-0.1745 rad) 实现仰角
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='launcher_tf',
            # 参数格式: x y z yaw pitch roll frame_id child_frame_id
            arguments=['0', '0', '0.05', '0', '-0.02', '0', 'camera_link', 'launcher_link']
        ),
        
        # # 额外补充：如果你需要把 camera_link 也挂在 map 上方便一起显示，可以加这个：
        # # (假设相机平视，高度 1.2m)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_base_tf',
        #     arguments=['0', '0', '1.2', '0', '0', '0', 'map', 'camera_link']
        # )
    ])