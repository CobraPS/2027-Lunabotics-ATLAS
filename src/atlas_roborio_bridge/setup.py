from setuptools import find_packages, setup

package_name = 'atlas_roborio_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyntcore==2026.2.2',
    ],
    zip_safe=True,
    maintainer='Priyesh Patel',
    maintainer_email='priyeshjpatel@pm.me',
    description='ROS 2 bridge for communication between the ATLAS Jetson and RoboRIO.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'roborio_bridge = atlas_roborio_bridge.roborio_bridge:main',
        ],
    },
)
