from setuptools import find_packages, setup

package_name = 'robot_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arda',
    maintainer_email='elibol24@itu.edu.tr',
    description='mode_manager, control_panel (PyQt5 teleop) and robot_console nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mode_manager = robot_tools.mode_manager:main',
            'control_panel = robot_tools.control_panel:main',
        ],
    },
)
