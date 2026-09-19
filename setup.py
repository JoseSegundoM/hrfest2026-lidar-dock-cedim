from glob import glob

from setuptools import setup

package_name = 'cedim_dock'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Equipo CEDIM',
    maintainer_email='JoseSegundoM@users.noreply.github.com',
    description='Docking autonomo del iRobot Create 3 guiado unicamente por LiDAR.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'docking_node = cedim_dock.docking_node:main',
        ],
    },
)
