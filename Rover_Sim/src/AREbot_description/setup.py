import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'AREbot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edar',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_relay = AREbot_description.scan_relay:main',
            'map_server = AREbot_description.map_server:main',
            'teleop_twist_keyboard = AREbot_description.teleop_twist_keyboard:main',
            'run_amcl = AREbot_description.run_amcl:main',
            'imu_reader = AREbot_description.imu_reader:main',
        ],
    },
)
