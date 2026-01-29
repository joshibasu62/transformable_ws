import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_tryd = get_package_share_directory('tryd')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    install_dir = os.path.dirname(pkg_tryd)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[install_dir]
    )

    world_file = os.path.join(pkg_tryd, 'worlds', 'world.sdf')
    urdf_file = os.path.join(pkg_tryd, 'urdf', 'tryd1.urdf')
    sdf_file = os.path.join(pkg_tryd, 'sdf', 'model.sdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 4. Robot State Publisher (Publishes TF tree)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        # launch_arguments={'gz_args': f'-r {world_file}'}.items(),  #Uncomment for custom world (if you want to change the world go to the world.sdf inside worlds)
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            # '-topic', 'robot_description',
            '-file', sdf_file,
            '-name', 'tryd',
            '-x', '0.0', '-y', '0.0', '-z', '0.5' 
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            "/trunk_imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output='screen'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_tryd, 'rviz', 'config.rviz')] # Uncomment if you have a config
    )

    return LaunchDescription([
        gz_resource_path,
        node_robot_state_publisher,
        node_joint_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        node_rviz
    ])