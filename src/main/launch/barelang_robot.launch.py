import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('main'),
        'config',
        'barelang.yaml'
        )

    motion_bridge_node=Node(
        package = 'motion_bridge',
        name = 'motion_bridge',
        executable = 'motion_bridge',
        parameters = [config],
        namespace = 'robot_1',
        output = 'screen'
    )

    main_strategy_node=Node(
        package = 'main',
        name = 'main_strategy',
        executable = 'main',
        parameters = [config],
        namespace = 'robot_1',
        output = 'screen'
    )

    game_controller_node=Node(
        package = 'game_controller',
        name = 'game_controller',
        executable = 'game_controller',
        parameters = [config],
        namespace = 'robot_1',
        output = 'screen'
    )

    ld.add_action(motion_bridge_node)
    ld.add_action(main_strategy_node)
    ld.add_action(game_controller_node)
    return ld