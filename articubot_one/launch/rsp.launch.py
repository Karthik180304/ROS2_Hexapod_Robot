import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription , ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    pkg_name = 'articubot_one'

    file_subpath = 'description/robot.urdf.xacro'
    # Process the URDF file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Process the URDF file using xacro
    xacro_args = ['xacro', '--inorder', xacro_file]
    xacro_process = ExecuteProcess(
        cmd=xacro_args,
        output='screen'
    )

     # Create a robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 
        	'use_sim_time': True}]
    )
    
    gazebo = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource([os.path.join(
    		get_package_share_directory('gazebo_ros'),'launch'), '/gazebo.launch.py']),
    	)
    	
    spawn_entity = Node(package='gazebo_ros', executable = 'spawn_entity.py',
    			arguments=['-topic', 'robot_description',
    				 '-entity', 'my_bot'],
    			output='screen')
    
    line_following_node = Node(
        package='hexapod_robot',
        executable='linefollower',
        output='screen'
    )

    leg_movement_node = Node(
        package="hexapod_robot",
        executable="leg_movement",
        output="screen"
    )

    leg_movement_demo_node = Node(
        package="hexapod_robot",
        executable="leg_movement_demo",
        output="screen"
    )
    controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['controller_spawner'],
        parameters=[{'update_rate': 50,
        	'use_sim_time': True,
        	'list_controllers': 'j_c1_lf_position_controller j_c1_rf_position_controller j_c1_lm_position_controller j_c1_rm_position_controller j_c1_lr_position_controller j_c1_rr_position_controller j_thigh_lf_position_controller j_thigh_rf_position_controller j_thigh_lm_position_controller j_thigh_rm_position_controller j_thigh_lr_position_controller j_thigh_rr_position_controller j_tibia_lf_position_controller j_tibia_rf_position_controller j_tibia_lm_position_controller j_tibia_rm_position_controller j_tibia_lr_position_controller j_tibia_rr_position_controller joint_state_controller'}]
    )
    # Launch!   
    return LaunchDescription([
    
        # xacro_process,
	    gazebo,
        # include_yaml,
        robot_state_publisher_node,
        spawn_entity,
        line_following_node,
        leg_movement_node,
        leg_movement_demo_node,
        controller_spawner_node
       
    ])