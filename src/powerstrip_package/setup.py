from setuptools import setup

package_name = 'powerstrip_package'

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
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Simulated Arduino and ROS2 powerstrip controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulated_arduino = powerstrip_package.simulated_arduino:main',
            'ros2_powerstrip = powerstrip_package.ros2_powerstrip:main',
        ],
    },
)
