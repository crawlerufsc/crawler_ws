from setuptools import setup

package_name = 'gps_interface'

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
    description='Node to interface a GPS Neo-6m over UART',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_pub = gps_interface.gps_pub:main',
        ],
    },
)
