import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('face_detector_cpp')
    config_file = os.path.join(pkg_share, 'config', 'face_detector_param.yaml')
    # cascade_file_path = os.path.join(pkg_share, 'moudles', 'haarcascade_frontalface_default.xml')
    prototxt_path = os.path.join(pkg_share, 'moudles', 'deploy.prototxt.txt')
    model_path = os.path.join(pkg_share, 'moudles', 'res10_300x300_ssd_iter_140000.caffemodel')

    face_detector_node = Node(
        package='face_detector_cpp',
        executable='face_detector_node',
        name='face_detector_node',
        output='screen',
        parameters=[config_file,
                    {'prototxt_path': prototxt_path},
                    {'model_path': model_path}]
    )

    return LaunchDescription([
        face_detector_node
    ])