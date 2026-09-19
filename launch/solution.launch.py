"""Lanza la solucion de acoplamiento del equipo CEDIM.

Este es el comando unico que pide la convocatoria:

    ros2 launch cedim_dock solution.launch.py

Da por hecho que el escenario oficial ya esta corriendo en otra terminal:

    ros2 launch create3_dock_challenge challenge_world.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('cedim_dock'), 'config', 'params.yaml'),
        description='Fichero de parametros del nodo de acoplamiento.'),
    DeclareLaunchArgument(
        'namespace', default_value='',
        description='Espacio de nombres del Create 3, si el escenario lo usa.'),
    DeclareLaunchArgument(
        'log_level', default_value='info',
        choices=['debug', 'info', 'warn', 'error'],
        description='Nivel de registro del nodo.'),
]


def generate_launch_description():
    docking_node = Node(
        package='cedim_dock',
        executable='docking_node',
        name='cedim_docking_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    return LaunchDescription(ARGUMENTS + [docking_node])
