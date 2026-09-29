"""Launch the rfn3d ROS 2 planner with every topic name and frame exposed as an argument.

Topic names are wired through remappings: the node keeps its canonical names and each is
remapped to whatever you pass. The frame is a node parameter instead, since a TF frame is
not a topic and cannot be remapped. Every default below is the value the node hardcodes
today, so launching with no overrides reproduces current behaviour. Override any of them
on the command line, e.g.:

    ros2 launch rfn3d planner.launch.py odom_topic:=/drone/odom frame_id:=map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, \
    PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# (arg name, default = the node's hardcoded topic, description). The default doubles as the
# remap source, so this list is the single source of truth for both the args and the remaps.
TOPICS = [
    ("odom_topic", "/odometry", "Odometry in (nav_msgs/Odometry)"),
    ("cloud_topic", "/drone_0/realsense/points", "Obstacle cloud in (sensor_msgs/PointCloud2)"),
    ("goal_topic", "/clicked_point", "Goal point in (geometry_msgs/PointStamped)"),
    ("trail_topic", "/trail_viz", "Travelled path out (nav_msgs/Path)"),
    ("ref_topic", "/traj_ref", "Tracking reference out (geometry_msgs/PointStamped)"),
    ("traj_viz_topic", "/traj_viz", "Trajectory markers out (visualization_msgs/MarkerArray)"),
    ("traj_topic", "/cmd_trajectory",
     "Committed trajectory out (trajectory_msgs/MultiDOFJointTrajectory)"),
]


def generate_launch_description():
    ld = LaunchDescription()

    for name, default, desc in TOPICS:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=desc))

    ld.add_action(DeclareLaunchArgument(
        "frame_id", default_value="world",
        description="TF frame the planner works and publishes in"))
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use the /clock topic instead of wall time"))
    ld.add_action(DeclareLaunchArgument(
        "initial_goal", default_value="",
        description="Publish a goal on startup, e.g. '0 0 1'. Empty = no goal."))

    planner = Node(
        package="rfn3d",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[{
            "frame_id": LaunchConfiguration("frame_id"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        remappings=[(default, LaunchConfiguration(name)) for name, default, _ in TOPICS],
    )
    ld.add_action(planner)

    goal = LaunchConfiguration("initial_goal")
    goal_topic = LaunchConfiguration("goal_topic")
    frame = LaunchConfiguration("frame_id")
    has_goal = PythonExpression(["'", goal, "' != ''"])

    publish_script = PathJoinSubstitution([
        FindPackageShare("rfn3d"), "scripts", "publish_goal.py",
    ])

    ld.add_action(TimerAction(period=3.0, actions=[
        ExecuteProcess(
            condition=IfCondition(has_goal),
            cmd=["python3", publish_script, goal_topic, frame, goal],
        ),
    ]))

    return ld
