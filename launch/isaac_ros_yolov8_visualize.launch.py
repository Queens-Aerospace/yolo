# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    my_package_dir = get_package_share_directory('isaac_ros_yolov8')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                my_package_dir, 'launch'),
                '/yolov8_tensor_rt.launch.py'])
        ),
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/zed_camera.launch.py']),
           launch_arguments={'camera_model': 'zedxm', 'publish_tf' : 'true', 'publish_map_tf': 'true', 'area_memory': 'false', 'path_max_count': '30', 'qos_depth': '5'}.items(),
       ),
         IncludeLaunchDescription(
           PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/yolov8_3d.launch.py']),
           launch_arguments={'input_image_topic': '/zed/zed_node/rgb_raw/image_raw_color', 'input_depth_topic' : '/zed/zed_node/depth/depth_registered', 'input_depth_info_topic': '/zed/zed_node/depth/camera_info'}.items(),
       ),
        #  Node(
        #     package='isaac_ros_yolov8',
        #     executable='image_resize.py',
        #     name='image_processor'
        # ),
        Node(
            package='isaac_ros_yolov8',
            executable='isaac_ros_yolov8_visualizer.py',
            name='yolov8_visualizer'
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_view',
            arguments=['/yolov8_processed_image']
        )
    ])
