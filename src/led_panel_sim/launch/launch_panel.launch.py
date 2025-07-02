from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the model root directory (i.e., the one containing `led_panel/`)
    model_root_path = os.path.join(os.getenv('HOME'), 'led_panel_ws', 'src', 'led_panel_sim', 'models')

    # Get the world file
    world_file = os.path.join(os.getenv('HOME'), 'led_panel_ws', 'src', 'led_panel_sim', 'worlds', 'led_panel.world')
    
    bridge_gz_ros = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_gz_ros',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/world/default/material_color@ros_gz_interfaces/msg/MaterialColor@gz.msgs.MaterialColor'
        ]
    )
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v 4', world_file],
            additional_env={
                'IGN_GAZEBO_RESOURCE_PATH': model_root_path,
                'GZ_SIM_RESOURCE_PATH': model_root_path,
            },
            output='screen'
        ),
        bridge_gz_ros,
    ])
