from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Set PMU analyzer config file environment variable
    pkg_share = get_package_share_directory('nodes_for_evaluation')
    pmu_config_path = os.path.join(pkg_share, 'config', 'pmu_config.yaml')
    set_pmu_config_env = SetEnvironmentVariable(
        'PMU_ANALYZER_CONFIG_FILE',
        pmu_config_path
    )

    # Declare launch argument for number of publishers
    num_publishers_arg = DeclareLaunchArgument(
        'num_publishers',
        default_value='2',
        description='Number of publishers (1-4)'
    )
    num_publishers = LaunchConfiguration('num_publishers')

    # Create publisher nodes dynamically
    publisher1 = Node(
        package='nodes_for_evaluation',
        executable='subscribe_publisher',
        name='subscribe_publisher_1',
        parameters=[{'topic_id': 1}],
        output='screen'
    )

    publisher2 = Node(
        package='nodes_for_evaluation',
        executable='subscribe_publisher',
        name='subscribe_publisher_2',
        parameters=[{'topic_id': 2}],
        output='screen'
    )

    publisher3 = Node(
        package='nodes_for_evaluation',
        executable='subscribe_publisher',
        name='subscribe_publisher_3',
        parameters=[{'topic_id': 3}],
        output='screen'
    )

    publisher4 = Node(
        package='nodes_for_evaluation',
        executable='subscribe_publisher',
        name='subscribe_publisher_4',
        parameters=[{'topic_id': 4}],
        output='screen'
    )

    # Subscriber node that accepts command line argument
    subscriber = Node(
        package='nodes_for_evaluation',
        executable='sync_subscriber',
        name='sync_subscriber',
        arguments=[num_publishers],
        output='screen'
    )

    # Start source publisher
    start_pub = Node(
        package='nodes_for_evaluation',
        executable='source_publisher',
        name='source_publisher',
        output='screen'
    )

    return LaunchDescription([
        set_pmu_config_env,
        num_publishers_arg,
        start_pub,
        publisher1,
        publisher2,
        publisher3,
        publisher4,
        subscriber
    ])
