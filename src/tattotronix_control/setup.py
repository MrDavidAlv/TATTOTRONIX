import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tattotronix_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'trajectories'),
         glob('config/trajectories/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario David Alvarez Vallejo',
    maintainer_email='ing.marioalvarezvallejo@gmail.com',
    description='ros2_control controller configuration and spawners for the TATTOTRONIX arm',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'draw_logo = tattotronix_control.draw_logo:main',
    ]},
)
