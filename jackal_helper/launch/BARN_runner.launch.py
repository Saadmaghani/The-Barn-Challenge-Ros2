import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.actions import LogInfo, Shutdown, TimerAction, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetRemap

"""Launch gz simulation, ros-gz-bridge, jackal, BARN simulation runner, and (rviz2/gz gui) optionally."""

# TODO: integrate use_sim_time throughout

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('gui', default_value='false',
                          choices=['true', 'false'], description='Start gazebo gui.'),
    DeclareLaunchArgument('world_idx', default_value='1',
                          description='BARN World Index: [0-299].'),
    DeclareLaunchArgument('setup_path',
                          default_value=PathJoinSubstitution([get_package_share_directory('jackal_helper'), 'config']),
                          description='Clearpath setup path. The folder which contains robot.yaml.'),
    DeclareLaunchArgument('out_file',
                          default_value="out.txt",
                          description='File name which trial results are stored.'),
    DeclareLaunchArgument('timeout',
                          default_value='100',
                          description='Trial timeout time in seconds.'),
]

def parse_world_idx(world_idx:str)->str:
    world_idx = int(world_idx)
    if world_idx < 300:  # static environment from 0-299
        world_name = f"BARN/world_{world_idx}.world" 
        GOAL_DIST = 10 
    elif world_idx < 360:  # Dynamic environment from 300-359
        world_name = f"DynaBARN/world_{world_idx - 300}.world"
        GOAL_DIST = 20 
    else:
        raise ValueError(f"World index {world_idx} does not exist")

    return world_name, GOAL_DIST

def launch_ros_gazebo(context, *args, **kwargs):
    """
    Launches Gazebo server + gui, sets up resources path, and launches ros-gz-bridge.
    """
    # Packages
    pkg_jackal_helper = get_package_share_directory('jackal_helper')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gui_config = PathJoinSubstitution([pkg_jackal_helper, 'config', 'gui.config'])

    gui_cmd = "" if LaunchConfiguration('gui').perform(context)=='true' else " -s"
    world_name = parse_world_idx(LaunchConfiguration("world_idx").perform(context))[0]
    world_path = os.path.join(pkg_jackal_helper, "worlds", world_name)
    
    # Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [world_path,
                        gui_cmd,
                         ' -r -v 0',
                         ' --gui-config ',
                         gui_config]),
        ]
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/control@ros_gz_interfaces/srv/ControlWorld',
            '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/robot/touched@std_msgs/msg/Bool@gz.msgs.Boolean',
            '/model/robot/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '--ros-args', '--log-level', 'WARN'
        ],
        output='screen'
    )

    return [gz_sim, clock_bridge]

def spawn_jackal(context, *args, **kwargs):
    # SPAWN ROBOT
    spawner_launch_path = PathJoinSubstitution([get_package_share_directory('clearpath_gz'), 'launch', 'robot_spawn.launch.py'])
    
    world_name = parse_world_idx(LaunchConfiguration('world_idx').perform(context))[0]
    remap_scan_topic = SetRemap(src='/sensors/lidar2d_0/scan', dst='/front/scan')

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([spawner_launch_path]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('world', world_name),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', '2.0'),
            ('y', '2.0'),
            ('z', '0.3')]
    )

    return [remap_scan_topic, robot_spawn]

def launch_navigation_stack(context, *args, **kwargs):

    # launch your navigation stack here
    nav2_launch_path = PathJoinSubstitution([get_package_share_directory('jackal_helper'), 'launch', 'nav2_bringup.launch.py'])
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_path]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('scan_topic', '/front/scan'),
            ('nav2_params_file', 'nav2.yaml'),
            ('log_level', 'WARN')
            ]
    )

    # Get goal distance from world_idx and make a string to send to the /navigate_to_pose action server
    relative_goal_distance = parse_world_idx(LaunchConfiguration("world_idx").perform(context))[1]
    goal = {
        "pose": {
            "header": {"frame_id": "odom"},
            "pose": {
                "position": {
                    "x": relative_goal_distance,
                    "y": 0.0,
                    "z": 0.0,
                },
                "orientation": {"w": 1.0},
            }
        }
    }
    goal_str = yaml.dump(goal, default_flow_style=True, width=float("inf")).rstrip("\n")

    # goal published after 10 seconds
    publish_goal = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="Publishing Nav2 goal..."),
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal',
                    '/navigate_to_pose',
                    'nav2_msgs/action/NavigateToPose',
                    goal_str
                ],
                output='screen'
            )
        ]
    )




    return [nav2_launch, publish_goal]

def generate_launch_description():

    # Start the BARN_runner node after 10 seconds. this replaces the run.py in ROS1 version of The_BARN_Challenge.
    BARN_runner_node = TimerAction(
        period = 10.0,
        actions=[
            LogInfo(msg="BARN Runner node started. Starting Trial..."),
            Node(
                package="jackal_helper",
                executable="barn_runner.py",
                output='screen',
                emulate_tty=True,
                parameters=[
                    {'world_idx': LaunchConfiguration('world_idx')},
                    {'out_file': LaunchConfiguration('out_file')},
                    {'timeout': LaunchConfiguration('timeout')}
                    ],
                on_exit=[
                    LogInfo(msg="Trial ended. Shutting down in 5 seconds..."),
                    TimerAction(period=5.0, actions=[Shutdown(), ExecuteProcess(cmd=['pkill','-f','\'gz sim\''], shell=True)]),
                    ],
                
            )
        ]
    ) 
    
    # launch navigation stack after X seconds
    nav_stack = TimerAction(
        period=15.0,
        actions=[LogInfo(msg="Launching Nav2..."), OpaqueFunction(function=launch_navigation_stack)]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_ros_gazebo))
    ld.add_action(OpaqueFunction(function=spawn_jackal))
    ld.add_action(BARN_runner_node)
    ld.add_action(nav_stack)
    return ld

