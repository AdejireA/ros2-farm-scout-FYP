from setuptools import find_packages, setup

package_name = 'teleop_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adejirea',
    maintainer_email='nunsiomi@gmail.com',
    description='Keyboard teleoperation node publishing to /cmd_vel_teleop',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = teleop_node.keyboard_teleop:main',
        ],
    },
)
