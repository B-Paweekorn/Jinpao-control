#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('jinpao_description')
    rviz_path = os.path.join(pkg,'config','test_display.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    

    robot_path_description = os.path.join(get_package_share_directory(
                                    'jinpao_description'), 
                                    'robot',
                                    'carver.xacro')

    robot_description = xacro.process_file(robot_path_description).toxml()
    
    robot_namespace = ''
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[
                                    {'robot_description': robot_description},
                                    {'frame_prefix': robot_namespace+'/'}
                                  ],
                                  namespace=robot_namespace,
                                  
    )
    
    joint_state_publisher = Node(package='joint_state_publisher',
                                    executable='joint_state_publisher',
                                    name='joint_state_publisher',
                                    namespace=robot_namespace
    )

    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                        executable='joint_state_publisher_gui',
                                        name='joint_state_publisher_gui',
                                        namespace=robot_namespace
    )
    launch_description = LaunchDescription()
    
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher)
    launch_description.add_action(joint_state_publisher_gui)
    
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    