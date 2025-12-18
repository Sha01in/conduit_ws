from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    factory_robot_node = Node(
        package='conduit_agent',
        executable='factory_robot',
        name='factory_robot',
        output='screen'
    )

    orchestrator_node = Node(
        package='conduit_agent',
        executable='orchestrator',
        name='orchestrator',
        output='screen'
    )
    
    # Delay orchestrator start by 2 seconds to ensure server is up
    delayed_orchestrator = TimerAction(
        period=2.0,
        actions=[orchestrator_node]
    )

    return LaunchDescription([
        factory_robot_node,
        delayed_orchestrator
    ])
