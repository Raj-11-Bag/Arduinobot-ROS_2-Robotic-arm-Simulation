from launch import LaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    
    arduinobot_description_dir= get_package_share_directory("arduinobot_description")

    model_arg = DeclareLaunchArgument(
        name= 'model',
        default_value=os.path.join(arduinobot_description_dir, "urdf" , "arduinobot.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value = [
            str(Path(arduinobot_description_dir).parent.resolve())
        ]
    )
    # find the file in the arduinobot_description folder and take meshes

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    

    robot_description = ParameterValue(Command([
        "xacro ", 
        LaunchConfiguration("model"),
        " is_ignition:=",
        is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time":True}]                     #it tell weather robot is running in somulation or not
    )

#this node is running the empty gazebo simulation
    gazebo = IncludeLaunchDescription(                     
        PythonLaunchDescriptionSource([ 
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch"
            ), "/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ("gz_args", [" -v 4 -r empty.sdf"]) #r = start visuvalization immegiatly, empty= background only, no element in it
        ]
    )
# now we simulate our robot in this gazebo for that we are creating new instance (node)
    gz_spwan_entity=TimerAction(
    period=5.0,
    actions=[
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-topic", "robot_description", "-name", "arduinobot"]
        )
    ]
)

#transmission of data from gazebo simulation to ros2
    gz_ros2_bridge= Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"
        ]
    )


    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        gazebo,
        robot_state_publisher,
        gz_spwan_entity,
        gz_ros2_bridge
    ])