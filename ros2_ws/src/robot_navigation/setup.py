from glob import glob
from setuptools import find_packages, setup

name = 'robot_navigation'
setup(name=name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml']),
                  ('share/' + name + '/config', glob('config/*')),
                  ('share/' + name + '/maps', glob('maps/*'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Robot simulation maintainers', maintainer_email='maintainer@example.com',
      description='Ground-truth Nav2 navigation for the warehouse', license='BSD-3-Clause',
      entry_points={'console_scripts': ['goal_selector = robot_navigation.goal_selector:main']})
