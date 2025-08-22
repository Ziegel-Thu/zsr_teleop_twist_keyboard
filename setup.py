from setuptools import setup

package_name = 'zsr_teleop_twist_keyboard'

setup(
    name=package_name,
    version='2.4.0',
    packages=[],
    py_modules=[
        'teleop_twist_keyboard',
        'zsr_teleop_twist_keyboard',
        'zsr_serial_publisher'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer= 'Ziegel Zheng',
    maintainer_email='zhengshurui0627@gmail.com',
    author='Ziegel Zheng',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A robot-agnostic teleoperation node to convert keyboard'
                'commands to Twist messages.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_twist_keyboard = teleop_twist_keyboard:main',
            'zsr_teleop_twist_keyboard = zsr_teleop_twist_keyboard:main',
            'zsr_serial_publisher = zsr_serial_publisher:main'
        ],
    },
)
