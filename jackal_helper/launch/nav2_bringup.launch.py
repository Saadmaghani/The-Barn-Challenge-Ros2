# Copyright (c) 2018 Intel Corporation
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

# Taken from https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/launch/navigation_launch.py
# Adapted to remove some servers

import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml



ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value=PathJoinSubstitution([get_package_share_directory('jackal_helper'), 'config']),
                          description='Clearpath setup path'),
    DeclareLaunchArgument('scan_topic',
                          default_value='',
                          description='Override the default 2D laserscan topic'),
    DeclareLaunchArgument('nav2_params_file',
                          default_value='nav2.yaml',
                          description='The nav2.yaml params file.'),
    DeclareLaunchArgument('log_level', 
                            default_value='info', 
                            description='log level')
]

def jackal_nav2_setup(context, *args, **kwargs):
    # setup nav2 at the clearpath's jackal level

    # Packages
    pkg_jackal_helper = get_package_share_directory('jackal_helper')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    scan_topic = LaunchConfiguration('scan_topic')

    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    # see if we've overridden the scan_topic
    eval_scan_topic = scan_topic.perform(context)
    if len(eval_scan_topic) == 0:
        eval_scan_topic = '/front/scan'

    file_parameters = PathJoinSubstitution([setup_path, LaunchConfiguration('nav2_params_file')])

    rewritten_parameters = RewrittenYaml(
        source_file=file_parameters,
        param_rewrites={
            # the only *.topic parameters are scan.topic, so rewrite all of them to point to
            # our desired scan_topic
            'topic': eval_scan_topic,
        },
        convert_types=True
    )
    
    return rewritten_parameters

    # launch_nav2 = PathJoinSubstitution(
    #   [pkg_jackal_helper, 'launch', 'nav2_bringup.launch.py'])

    # nav2 = GroupAction([
    #     SetRemap('/odom', '/platform/odom'),
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(launch_nav2),
    #         launch_arguments=[
    #             ('use_sim_time', use_sim_time),
    #             ('params_file', rewritten_parameters),
    #             ('use_composition', 'False'),
    #           ]
    #     ),
    # ])

    # return nav2

def nav2_bringup_setup(context, *args, **kwargs):
    # get parameter file from jackal_setup
    jackal_rewritten_parameters = jackal_nav2_setup(context)

    
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch Configurations & fixed parameters
    namespace = '' #LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    set_autostart_lc = SetLaunchConfiguration('autostart', 'True') #LaunchConfiguration('autostart')
    autostart = LaunchConfiguration('autostart')
    params_file = jackal_rewritten_parameters #LaunchConfiguration('params_file')
    # use_composition = 'False' #LaunchConfiguration('use_composition')
    container_name = 'nav2_container' #LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = False #LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        # 'route_server',
        'behavior_server',
        'velocity_smoother',
        # 'collision_monitor',
        'bt_navigator',
        # 'waypoint_follower',
        # 'docking_server',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/odom', '/platform/odom')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    
    load_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings
                + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
            ),
        ],
    )

    return [set_autostart_lc, stdout_linebuf_envvar, load_nodes]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=nav2_bringup_setup))
    return ld
