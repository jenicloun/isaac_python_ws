from setuptools import setup

package_name = 'custom_message'

setup(
    name=package_name,
    version='0.0.0',
    packages=['custom_message'],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac Sim',
    maintainer_email='isaac-sim-maintainers@nvidia.com',
    description='Custom Message Sample Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_name_remap_node = custom_message.joint_name_remap_node:main',
        ],
    },
)