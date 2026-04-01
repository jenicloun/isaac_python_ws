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
        ('share/' + package_name + '/launch', [
            'launch/mapf_demo.launch.py',
            'launch/mapf_single_demo.launch.py',
            'launch/mapf_single_nav_demo.launch.py',
            'launch/mapf_multi_nav_demo.launch.py',
            'launch/mapf_multi_goal_demo.launch.py',
            'launch/mapf_multi_goal_change_demo.launch.py',
            'launch/mapf_goal_demo.launch.py',
            'launch/mapf_multi_nav_demo_0318.launch.py',
            'launch/mapf_multi_nav_demo_0319.launch.py',
            'launch/total_navigation.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/mapf_params.yaml',
            'config/mapf_params_3g.yaml',
            'config/costmap_params.yaml',
            'config/carter_warehouse_navigation.yaml',
            'config/carter_warehouse_navigation.png',
            'config/carter_warehouse_navigation_005.yaml',
            'config/carter_warehouse_navigation_005.png',
            'config/multi_goals.yaml'
        ]),
        ('share/' + package_name + '/launch', [
            'launch/nav2/carter1_nav2_bringup.launch.py',
            'launch/nav2/multi_nav2_bringup.launch.py',
        ]),
        ('share/' + package_name + '/config/nav2', [
            'config/nav2/carter1_nav2_params.yaml',
            'config/nav2/carter2_nav2_params.yaml',
            'config/nav2/carter3_nav2_params.yaml',
        ]),
        ('share/' + package_name + '/config/amcl', [
            'config/amcl/carter1_amcl_params.yaml',
            'config/amcl/carter2_amcl_params.yaml',
            'config/amcl/carter3_amcl_params.yaml',
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='baek',
    maintainer_email='54278901+ivory234@g.skku.edu',
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