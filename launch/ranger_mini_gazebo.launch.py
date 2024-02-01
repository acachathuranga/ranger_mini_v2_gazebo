# BSD 2-Clause License
#
# Author : Achala Athukorala (chhathuranga@gmail.com)
# Copyright (c) 2024, Singapore University of Technology and Design
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
import xacro
import os

# Add boolean commands if true
def _boolean_arg(arg):
    cmd = ['"--', arg, '" if "True" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd

def generate_launch_description():
    # Get launch directory
    package_dir = get_package_share_directory('ranger_mini_v2_description')
    launch_dir = os.path.join(package_dir, 'launch')
    model_file = os.path.join(package_dir, 'urdf', 'ranger_mini_v2.urdf')
    # worlds_dir = os.path.join(get_package_share_directory('ssugv_gazebo'), 'worlds')

    # Declare the Launch Arguments
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value="ranger_mini",
        description='Robot Name (Namespace)')

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    dedeclare_rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'urdf_config.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=model_file,
        description='Absolute path to robot urdf file')

    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value="",
        # default_value=os.path.join(worlds_dir, 'willow_garage.sdf'),
        description='Full path to world model file to load')
    
    declare_pause_arg = DeclareLaunchArgument(
        'pause',
        default_value='False',
        description='Start simulation in paused state'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    
    declare_verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='True',
        description='Enable debug output (verbose)'
    )
    
    # Create the launch configuration variables
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pause = LaunchConfiguration('pause')
    use_sim_time = LaunchConfiguration('use_sim_time')
    verbose = LaunchConfiguration('verbose')
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/robot_description', 'robot_description')]
        
    # Specify the actions
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        remappings=remappings,  
            # remappings is required for previous ros2 versions 
            # that doesnt support 'frame_prefix' for tf namespacing
        parameters=[{'robot_description': Command(['xacro ', model_file])}]
    )
    
    
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', _boolean_arg('verbose'), 
             _boolean_arg('pause'), 
             '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', 
             world],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(['not ', headless])),
        cmd=['gzclient', 
             _boolean_arg('verbose')],
        cwd=[launch_dir], output='screen')

    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=[TextSubstitution(text='spawn_robot_'), robot_name],
        namespace=robot_name,
        arguments=['-entity', robot_name, '-topic', 'robot_description'],
        output='screen'
    )
    # Create the launch description and populate initial arguments
    launch_description = LaunchDescription()

    launch_description.add_action(declare_robot_name_arg)
    launch_description.add_action(declare_use_rviz_arg)
    launch_description.add_action(dedeclare_rviz_config_file_arg)
    launch_description.add_action(declare_world_arg)
    launch_description.add_action(declare_headless_arg)
    launch_description.add_action(declare_model_arg)
    launch_description.add_action(declare_pause_arg)
    launch_description.add_action(declare_use_sim_time_arg)
    launch_description.add_action(declare_verbose_arg)
    launch_description.add_action(start_robot_state_publisher_cmd)
    launch_description.add_action(start_gazebo_server_cmd)
    launch_description.add_action(start_gazebo_client_cmd)
    launch_description.add_action(spawn_robot_cmd)
    return launch_description
