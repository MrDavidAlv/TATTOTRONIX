# Copyright 2026 Mario David Alvarez Vallejo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tattotronix_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario David Alvarez Vallejo',
    maintainer_email='ing.marioalvarezvallejo@gmail.com',
    description='One entry point that draws on the simulated or the real TATTOTRONIX arm',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)
