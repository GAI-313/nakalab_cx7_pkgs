#!/usr/bin/env python3
from crane_x7_description.robot_description_loader import RobotDescriptionLoader

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

from ament_index_python.packages import get_package_share_directory

import os
import yaml
#import subprocess
import xacro
import traceback

from moveit_configs_utils import MoveItConfigsBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    try:
        ld = LaunchDescription()
        description_loader = RobotDescriptionLoader()

        robot_description_semantic_config = load_file(
            'cx7_config', 'config/crane_x7_d435.srdf')
        robot_description_semantic = {
            'robot_description_semantic': robot_description_semantic_config}

        kinematics_yaml = load_yaml('crane_x7_moveit_config', 'config/kinematics.yaml')

        # load by moveit_plugin
        '''
        moveit_config = (
            MoveItConfigsBuilder("cx7_config")
            .robot_description(file_path="urdf/crane_x7.urdf.xacro")
            .joint_limits(file_path="config/joint_limits.yaml")
            .to_moveit_configs()
        )
        '''

        # declare config
        declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description=('Set true when using the gazebo simulator.')
        )

        declare_frame_id = DeclareLaunchArgument(
            'frame_id', default_value='crane_x7_gripper_base_link',
            description=('Set true when using the gazebo simulator.')
        )

        move_to_pose = Node(package='cx7_controller',
                            executable='move_to_pose',
                            parameters=[{'robot_description': description_loader.load()},
                                        robot_description_semantic,
                                        kinematics_yaml])

        ''' devel
        servo_cmd_vel = Node(package='cx7_controller',
                            executable='servo_cmd_vel',
                            name='servo_cmd_vel',
                            parameters=[{'robot_description':rd_xacro},
                                        {'frame_id':LaunchConfiguration('frame_id')}])
        '''
        
        ld.add_action(declare_use_sim_time)
        ld.add_action(declare_frame_id)
        ld.add_action(SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),)
        ld.add_action(move_to_pose)
        #ld.add_action(servo_cmd_vel)

        return ld

    except:
        traceback.print_exc()
