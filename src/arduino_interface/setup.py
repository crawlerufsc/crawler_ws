import os
from glob import glob
from setuptools import setup

package_name = 'arduino_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bettanin',
    maintainer_email='gabrielnalinb@gmail.com',
    description='TODO: Package that interfaces with an Arduino Board via i2C',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_pub = arduino_interface.arduino_pub:main',
        ],
    },
)
