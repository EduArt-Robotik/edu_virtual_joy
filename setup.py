from distutils.core import setup

setup(
    version='0.1.0',
    scripts=['scripts/edu_virtual_joy_node.py'],
    packages=['edu_virtual_joy'],
    package_dir={'': 'src'},
    data_files=[
        ('share/edu_virtual_joy/images', [
            'images/arrow_curved_small.png',
            'images/arrow_straight_small.png',
            'images/gui_mecanum.png',
            'images/gui_skid.png',
            'images/iotbot_mecanum_top_vga.png',
            'images/iotbot_offroad_top_vga.png',
            'images/Logo_A_32.png',
            'images/Logo_Edu_100.png',
        ])
    ]
)
