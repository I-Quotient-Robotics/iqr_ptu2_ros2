from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    ptu2 = Node(
        package='iqr_ptu2',
        executable='node.py',
        name='ptu2_node',
    )
    model = Node(
        output='screen',
        package='iqr_ptu2',
        executable='model.py',
        name='ptu2_model'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ptu2_robot_state_publisher',
        arguments=[get_package_share_directory('iqr_ptu2') + '/urdf/ptu2.urdf'],
        namespace="ptu2",
    )
    return LaunchDescription([
        ptu2,
        model,
        robot_state_publisher,
    ])
