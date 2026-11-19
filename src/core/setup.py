from glob import glob
from setuptools import setup

setup(name='core', version='0.1.0',
      packages=['core', 'core.missions', 'core.behaviors', 'jetracer_ros2'],
      package_dir={'core': 'core', 'core.behaviors': '../line_slow',
                   'jetracer_ros2': 'vendor/jetracer_ros2'},
      data_files=[('share/ament_index/resource_index/packages', ['resource/core']),
                  ('share/core', ['package.xml']),
                  ('share/core/launch', glob('launch/*.launch.py')),
                  ('share/core/config', glob('config/*.yaml') + glob('config/*.json'))],
      install_requires=['setuptools'], zip_safe=False,
      entry_points={'console_scripts': ['core.core = core.core:main',
                                        'lane = core.runners:lane',
                                        'team_lane = core.runners:team_lane',
                                        'perception = core.runners:perception',
                                        'mission_01_slow = core.individual:slow',
                                        'mission_02_guide = core.individual:guide',
                                        'mission_03_pedestrian = core.individual:pedestrian',
                                        'mission_04_obstacle = core.individual:obstacle',
                                        'mission_05_intersection = core.individual:intersection',
                                        'mission_06_cone = core.individual:cone',
                                        'mission_07_traffic = core.individual:traffic',
                                        'mission_08_parking = core.individual:parking']})
