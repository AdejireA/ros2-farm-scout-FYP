from setuptools import find_packages, setup

package_name = 'mode_arbitration'

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
    description='Mode-switching arbitration between autonomous and manual control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arbitration_node = mode_arbitration.arbitration_node:main',
        ],
    },
)
