import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set the path to different files and folders
    pkg_share = FindPackageShare(package="ros2_light").find("ros2_light")
    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")   
    model_path = os.path.join(pkg_share, "description/robot.urdf")
    rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")

    # Launch configuration variables specific to simulation
    model = LaunchConfiguration("model", default=model_path)                                # Absolute path to robot urdf file
    rviz_config_file = LaunchConfiguration("rviz_config_file", default=rviz_config_path)    # Full path to the RVIZ config file
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub", default="True")        # Start robot state publisher if true
    use_joint_state_pub = LaunchConfiguration("use_joint_state_pub", default="False")       # Start joint_state_publisher if true (Set to false if using Gazebo diff_drive)
    use_rviz = LaunchConfiguration("use_rviz", default="False")                             # Start RVIZ if true
    use_gazebo = LaunchConfiguration("use_gazebo", default="True")                          # Start Gazebo if true
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")                      # Use simulation (Gazebo) clock if true

    # Subscribe to the joint states of the robot
    start_robot_state_publisher = Node(
        condition = IfCondition(use_robot_state_pub),
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"use_sim_time": use_sim_time, "robot_description": Command(["xacro ", model])}],
        arguments = [model_path]
    )
    start_joint_state_publisher = Node(
        condition = IfCondition(use_joint_state_pub),
        package = "joint_state_publisher",
        executable = "joint_state_publisher",
    )

    # Launch RViz
    start_rviz = Node(
        condition = IfCondition(use_rviz),
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments = ["-d", rviz_config_file]
    )    

    # Launch Gazebo
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")]),
        condition = IfCondition(use_gazebo),
    )

    # Spawm entity
    spawn_entity = Node(
        condition = IfCondition(use_gazebo),
        package="gazebo_ros", 
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity",  "my_robot"],
        output="screen",
    )

    # Launch Monitor
    start_monitor = Node(
        package="ros2_light", 
        executable="monitor.py",
    )

    # Launch Joystick
    start_joystick = Node(
        package="ros2_light", 
        executable="teleop_joy.py",
    )

    # Create the launch description
    return LaunchDescription([
        start_robot_state_publisher,
        start_joint_state_publisher,
        start_rviz,
        start_gazebo,
        spawn_entity,
        start_monitor,
        start_joystick,
    ])
