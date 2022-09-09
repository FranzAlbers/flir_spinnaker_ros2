# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from ament_index_python.packages import get_package_share_directory

camera_params_front = {
    'debug': False,
    'compute_brightness': True,
    'dump_node_map': False,
    # set parameters defined in chameleon.cfg    
    #'video_mode': 1,
    'image_width': 2048,
    'image_height': 1152,
    'offset_x': 0,
    'offset_y': 384,
    #'image_width': 2048,
    #'image_height': 1536,
    #'offset_x': 0,
    #'offset_y': 0,
    'pixel_format': 'RGB8', # 'BayerRG8, 'RGB8' or 'Mono8'
    #'pixel_coding': 'RGBPacked',
    'gain_auto': 'Continuous',
    #'exposure_auto': 'Continuous',
    'frame_rate_auto': 'Off', # 'Off' or 'Continuous'
    'frame_rate_enable': True,
    #'frame_rate_continous': True,
    'frame_rate': 20.0,
    'trigger_mode': 'Off',
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
    }

# Crop other side of the image on rear camera because it is rotated
camera_params_rear = camera_params_front.copy()
camera_params_rear['offset_y'] = 100


def generate_launch_description():
    """Create synchronized stereo camera."""
    flir_dir = get_package_share_directory('flir_spinnaker_ros2')
    config_dir = flir_dir + '/config/'
    container = ComposableNodeContainer(
            name='stereo_camera_container',
            namespace='camera',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='flir_spinnaker_ros2',
                    plugin='flir_spinnaker_ros2::CameraDriver',
                    name=LaunchConfig('camera_front'),
                    namespace='/sensing/camera',
                    parameters=[camera_params_front,
                                {'parameter_file': config_dir + 'chameleon_rst.cfg',
                                 'frame_id': 'camera_front/camera_optical_link',
                                 'serial_number': '18497292'}],
                    remappings=[('~/control', 'camera_front/exposure_control/control'),],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='flir_spinnaker_ros2',
                    plugin='flir_spinnaker_ros2::CameraDriver',
                    name=LaunchConfig('camera_rear'),
                    namespace='/sensing/camera',
                    parameters=[camera_params_rear,
                                {'parameter_file': config_dir + 'chameleon_rst.cfg',
                                 'frame_id': 'camera_rear/camera_optical_link',
                                 'serial_number': '18497287'}],
                    remappings=[('~/control', 'camera_rear/exposure_control/control'), ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_front_cam_node',
                    namespace='/sensing/camera/camera_front',
                    remappings=[('image', 'image_raw'),],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                # ComposableNode(
                #     package='cam_sync_ros2',
                #     plugin='cam_sync_ros2::CamSync',
                #     name='sync',
                #     namespace='/sensing/camera',
                #     parameters=[],
                #     extra_arguments=[{'use_intra_process_comms': True}],
                # ),
                # ComposableNode(
                #     package='exposure_control_ros2',
                #     plugin='exposure_control_ros2::ExposureControl',
                #     name='exposure_control',
                #     namespace='/sensing/camera/camera_front',
                #     parameters=[{'cam_name': LaunchConfig('camera_front'),
                #                  'max_gain': 20.0,
                #                  'gain_priority': False,
                #                  'brightness_target': 100,
                #                  'max_exposure_time': 9500.0,
                #                  'min_exposure_time': 1000.0}],
                #     remappings=[('~/meta', ['/', LaunchConfig('camera_front'),
                #                             '/meta']), ],
                #     extra_arguments=[{'use_intra_process_comms': True}],
                # ),
            ],
            output='screen',
    )

    # Workaround for keeping rate up and delay low. Should also contain the camera_rear driver node, but this is not possible for now
    # https://github.com/berndpfrommer/flir_spinnaker_ros2/issues/24
    container_rear_cam_proc = ComposableNodeContainer(
        name='camera_rear_proc_container',
        namespace='camera',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_rear_cam_node',
                namespace='/sensing/camera/camera_rear',
                remappings=[('image', 'image_raw'),],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='image_flip',
                plugin='image_flip::ImageFlipNode',
                name='rotate_rear_cam_node',
                namespace='/sensing/camera/camera_rear',
                parameters=[{'rotation_steps': 2},
                            {'use_camera_info': False}],
                remappings=[('image', 'image_rect'),],
                #extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    name_0_arg = LaunchArg('camera_front', default_value=['camera_front'],
                           description='name of camera 0')
    name_1_arg = LaunchArg('camera_rear', default_value=['camera_rear'],
                           description='name of camera 1')
    return launch.LaunchDescription([name_0_arg, name_1_arg, container, container_rear_cam_proc])
