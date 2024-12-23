from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

def generate_launch_description():
    gazebo_default = True
    gazebo_robot_name_default = 'robotis_op3'

    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    action_file_path_default = get_package_share_directory('op3_action_module') + '/data/motion_4095.bin'
    device_name_default = '/dev/ttyUSB0'


    return LaunchDescription([
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'op3_action_editor', 'op3_action_editor',
                  '--ros-args',
                  '-p', f'gazebo:={gazebo_default}',
                  '-p', f'gazebo_robot_name:={gazebo_robot_name_default}',
                  '-p', f'offset_file_path:={offset_file_path_default}',
                  '-p', f'robot_file_path:={robot_file_path_default}',
                  '-p', f'init_file_path:={init_file_path_default}',
                  '-p', f'action_file_path:={action_file_path_default}',
                  '-p', f'device_name:={device_name_default}'
                ],
            # cmd=['ros2', 'run', 'op3_action_editor', 'op3_action_editor',
            #     '--ros-args',
            #     '-p', f'gazebo:={gazebo_default}',
            #     '-p', f'gazebo_robot_name:={gazebo_robot_name_default}',
            #     '-p', f'offset_file_path:={offset_file_path_default}',
            #     '-p', f'robot_file_path:={robot_file_path_default}',
            #     '-p', f'init_file_path:={init_file_path_default}',
            #     '-p', f'action_file_path:={action_file_path_default}',
            #     '-p', f'device_name:={device_name_default}'
            #   ],
            output='screen',
#            shell=True,
        )
    ])

