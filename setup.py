from setuptools import setup

package_name = 'edu_virtual_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/images', [
            'images/arrow_curved_small.png',
            'images/arrow_straight_small.png',
            'images/gui_mecanum.png',
            'images/gui_skid.png',
            'images/iotbot_mecanum_top_vga.png',
            'images/iotbot_offroad_top_vga.png',
            'images/Logo_A_32.png',
            'images/Logo_Edu_100.png',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christian Wendt',
    maintainer_email='christian.wendt@eduart-robotik.com',
    description='This package comprises a ROS joy interface for a keyboard steering concept.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_joy = edu_virtual_joy.edu_virtual_joy_node:main'
        ],
    },
)
