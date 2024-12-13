import subprocess
from ament_index_python.packages import get_package_share_directory

def main():
    # Define your package and executable
    package = "op3_action_editor"
    executable = "op3_action_editor"

    gazebo_default = True
    gazebo_robot_name_default = 'robotis_op3'

    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    action_file_path_default = get_package_share_directory('op3_action_module') + '/data/motion_4095.bin'
    device_name_default = '/dev/ttyUSB0'


    # Define any parameters or arguments
    params = [
        '--ros-args',
        '-p', f'gazebo:={gazebo_default}',
        '-p', f'gazebo_robot_name:={gazebo_robot_name_default}',
        '-p', f'offset_file_path:={offset_file_path_default}',
        '-p', f'robot_file_path:={robot_file_path_default}',
        '-p', f'init_file_path:={init_file_path_default}',
        '-p', f'action_file_path:={action_file_path_default}',
        '-p', f'device_name:={device_name_default}'
    ]

    try:
        # Run the node in the same terminal
        subprocess.run(['ros2', 'run', package, executable] + params, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error while running node: {e}")

if __name__ == "__main__":
    main()
