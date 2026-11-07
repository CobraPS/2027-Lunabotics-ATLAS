from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'atlas_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
            [path for path in glob('worlds/*') if os.path.isfile(path)]),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'models', 'atlas'),
            glob('models/atlas/*.sdf') + glob('models/atlas/*.xacro'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ATLAS Team',
    maintainer_email='93223640+CobraPS@users.noreply.github.com',
    description='Provisional standalone ATLAS Gazebo Harmonic simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'command_watchdog = atlas_navigation.command_watchdog:main',
            'cloud_to_scan = atlas_navigation.cloud_to_scan:main',
        ],
    },
)
