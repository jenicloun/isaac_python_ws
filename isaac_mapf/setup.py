import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'isaac_mapf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 1. launch .launch.py
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # 2. launch/nav2 .py
        (os.path.join('share', package_name, 'launch/nav2'), glob('launch/nav2/*.launch.py')),

        # 3. config .yaml, .png
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),

        # 4. config/nav2 .yaml
        (os.path.join('share', package_name, 'config/nav2'), glob('config/nav2/*.yaml')),
        
        # 5. config/amcl .yaml 
        (os.path.join('share', package_name, 'config/amcl'), glob('config/amcl/*.yaml'))
        

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenicloun',
    maintainer_email='jenicloun@hanyang.ac.kr',
    description='Isaac MAPF demo package',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'tf_prefix_relay = isaac_mapf.tf_prefix_relay:main',
            'goal_pose_publisher = isaac_mapf.goal_pose_publisher:main',
            'plan_to_cmdvel = isaac_mapf.plan_to_cmdvel:main',
            'path_bridge = isaac_mapf.path_bridge:main',
            'mapf_goal_checker = isaac_mapf.mapf_goal_checker:main',
        ],
    },
)