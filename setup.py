from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'BLEutms'

setup(
    name='BLEutms',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tento',
    maintainer_email='tentoshinz@gmail.com',
    description='BLEutms',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'BLEutms_sub = BLEutms.BLEutms_sub:main',
            'BLEutms = BLEutms.BLEutms:main',
            'BLEutms_display = BLEutms.BLEutms_display:main'
        ],
    },
)
