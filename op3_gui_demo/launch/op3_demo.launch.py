from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    
    op3_gui_demo_pkg_path = FindPackageShare('op3_gui_demo')
    demo_config_path = PathJoinSubstitution([op3_gui_demo_pkg_path, 'config', 'gui_config.yaml'])
    op3_gui_demo_node = Node(
            package='op3_gui_demo', 
            executable='op3_gui_demo', 
            output='screen',
            parameters=[{"demo_config": demo_config_path}],
            remappings=[('/op3_demo/ik_target_pose', '/pose_panel/pose')],
    )
   
    ld.add_action(op3_gui_demo_node)

    return ld
