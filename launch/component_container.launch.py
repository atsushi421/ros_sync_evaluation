from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    # Get the number of publishers from launch configuration
    num_publishers = int(LaunchConfiguration('num_publishers').perform(context))
    sync_policy = LaunchConfiguration('sync_policy').perform(context)
    max_interval_duration = float(LaunchConfiguration('max_interval_duration').perform(context))

    # Create composable nodes list
    composable_nodes = [
        # SourcePublisher
        ComposableNode(
            package='nodes_for_evaluation',
            plugin='SourcePublisher',
            name='source_publisher',
        ),
    ]

    # Add SubscribePublisher nodes based on num_publishers
    for i in range(1, num_publishers + 1):
        composable_nodes.append(
            ComposableNode(
                package='nodes_for_evaluation',
                plugin='SubscribePublisher',
                name=f'subscribe_publisher_{i}',
                parameters=[{'topic_id': i}],
            )
        )

    # Add SyncSubscriber component with appropriate template
    sync_plugin = f'SyncSubscriber<{num_publishers}>'
    composable_nodes.append(
        ComposableNode(
            package='nodes_for_evaluation',
            plugin=sync_plugin,
            name='sync_subscriber',
            parameters=[{
                'sync_policy': sync_policy,
                'max_interval_duration': max_interval_duration
            }],
        )
    )

    # Create container
    container = ComposableNodeContainer(
        name='sync_evaluation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    return [container]


def generate_launch_description():
    # Declare launch arguments
    num_publishers_arg = DeclareLaunchArgument(
        'num_publishers',
        default_value='2',
        description='Number of publishers (1-8)'
    )

    sync_policy_arg = DeclareLaunchArgument(
        'sync_policy',
        default_value='exact',
        description='Synchronization policy: "exact" or "approximate"'
    )

    max_interval_duration_arg = DeclareLaunchArgument(
        'max_interval_duration',
        default_value='100.0',
        description='Maximum time interval for approximate synchronization (milliseconds)'
    )

    return LaunchDescription([
        num_publishers_arg,
        sync_policy_arg,
        max_interval_duration_arg,
        OpaqueFunction(function=launch_setup),
    ])
