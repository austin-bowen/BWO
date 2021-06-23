from setuptools import setup

package_name = 'bwo'

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
    maintainer='Austin Bowen',
    maintainer_email='austin.bowen.314@gmail.com',
    description='ROS2 Foxy package for my robot BWO.',
    license='The MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = bwo.camera_node:main',
            'drive_motors = bwo.drive_motor_node:main',
            'object_detect = bwo.object_detect_node:main'
        ],
    },
)
