from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    gazebo_default = True
    gazebo_robot_name_default = 'robotis_op3'

    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    action_file_path_default = get_package_share_directory('op3_action_module') + '/data/motion_4095.bin'
    device_name_default = '/dev/ttyUSB0'

    action_editor_node = Node(
        package='op3_action_editor', 
        executable='op3_action_editor', 
        output='screen',
        parameters=[{
            'gazebo': gazebo_default,
            'gazebo_robot_name': gazebo_robot_name_default,
            'offset_file_path': offset_file_path_default,
            'robot_file_path': robot_file_path_default,
            'init_file_path': init_file_path_default,
            'action_file_path': action_file_path_default,
            'device_name': device_name_default
        }]
    )

    ros_mpg321_player = Node(
            package='ros_mpg321_player',
            executable='ros_mpg321_player',
            output='screen'
        )

    ld.add_action(action_editor_node)
    #ld.add_action(ros_mpg321_player)

    return ld
