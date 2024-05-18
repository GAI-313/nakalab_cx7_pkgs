#!/usr/bin/env python3
## launcher
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
## declare configs
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
## ext modules
from launch.conditions import IfCondition, UnlessCondition

import os
import traceback

def generate_launch_description():
    ld = LaunchDescription()

    # prefix
    crane_x7_gazebo_prefix = get_package_share_directory('crane_x7_gazebo')
    crane_x7_examples_prefix = get_package_share_directory('crane_x7_examples')

    # launch args
    declare_simulation = DeclareLaunchArgument('is_sim', default_value='true',
                                        description='Set this parameter to true when using simulation')
    declare_use_d435 = DeclareLaunchArgument('use_d435', default_value='false',
                                        description='If you have a D435 camera mounted, set this value to true')
    
    try:
        # exec simulation
        sim_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            #[crane_x7_gazebo_prefix, '/launch', '/crane_x7_with_table.launch.py']
                            [ThisLaunchFileDir(), '/crane_x7_d435_with_table.launch.py']
                        ),
                        condition=IfCondition(LaunchConfiguration('is_sim'))
                    )

        # exec real robot
        quick_launch = IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                [crane_x7_examples_prefix, '/launch', '/example.launch.py']
                            ),
                            condition=UnlessCondition(LaunchConfiguration('is_sim'))
                        )

        ## configs
        ld.add_action(declare_simulation)
        ld.add_action(declare_use_d435)

        ## action launcher
        ld.add_action(sim_launch)
        ld.add_action(quick_launch)

        return ld
        
    except:
        traceback.print_exc()
