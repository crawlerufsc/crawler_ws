from setuptools import setup

package_name = 'imu_interface3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bettanin',
    maintainer_email='gabrielnalinb@gmail.com',
    description='Node to interface a MPU6050 over i2c',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_pub = imu_interface3.imu_pub:main',
        ],
    },
)
