from glob import glob
from setuptools import find_packages, setup

name = 'robot_control'
setup(name=name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml']),
                  ('share/' + name + '/launch', glob('launch/*.launch.py')),
                  ('share/' + name + '/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Robot simulation maintainers', maintainer_email='maintainer@example.com',
      description='Robot control and terminal teleoperation', license='BSD-3-Clause',
      entry_points={'console_scripts': ['teleop = robot_control.nodes.teleop:main',
                                       'waypoint = robot_control.nodes.waypoint:main']})
