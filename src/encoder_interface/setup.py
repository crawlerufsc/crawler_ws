from setuptools import setup

package_name = 'encoder_interface'

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
    description='Node to interface a AS5600 over i2c',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_pub = encoder_interface.encoder_pub:main',
        ],
    },
)
