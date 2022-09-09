from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

camera_params = {
    'frame_id': 'camera_rear/camera_optical_link',
    'debug': False,
    'compute_brightness': False,
    'dump_node_map': False,
    # set parameters defined in chameleon.cfg    
    #'video_mode': 1,
    'image_width': 2048,
    'image_height': 1152,
    'offset_x': 0,
    'offset_y': 100,
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

def generate_launch_description():
    """launch chameleon camera node."""
    flir_dir = get_package_share_directory('flir_spinnaker_ros2')
    config_dir = flir_dir + '/config/'
    name_arg = LaunchArg('camera_name', default_value='camera_rear',
                         description='camera name')
    serial_arg = LaunchArg('serial', default_value="'18497287'",
                           description='serial number')
    print([LaunchConfig('serial'),'_'])
    node = Node(package='flir_spinnaker_ros2',
                executable='camera_driver_node',
                output='screen',
                name=[LaunchConfig('camera_name')],
                namespace='/sensing/camera',
                parameters=[camera_params,
                            {'parameter_file': config_dir + 'chameleon_rst.cfg',
                             'serial_number': [LaunchConfig('serial')],
                            }],
                #remappings=[('~/control', '/exposure_control/control'),],
    )
    return LaunchDescription([name_arg, serial_arg, node])
