from glob import glob
from setuptools import find_packages, setup

name = 'robot_bringup'
setup(name=name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml']),
                  ('share/' + name + '/launch', glob('launch/*.launch.py')),
                  ('share/' + name + '/config', glob('config/*.yaml') + glob('config/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Robot simulation maintainers', maintainer_email='maintainer@example.com',
      description='Launch and configure robot simulation', license='BSD-3-Clause',
      entry_points={'console_scripts': ['rviz_view = robot_bringup.rviz_view:main']})
