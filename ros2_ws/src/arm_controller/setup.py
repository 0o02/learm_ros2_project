from setuptools import find_packages, setup

package_name = 'arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ubunturos',
    maintainer_email='ubunturos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_action_server = arm_controller.serial_action_server:main',
            'realtime_serial_controller = arm_controller.realtime_serial_controller:main',
            'serial_action_server_binary = arm_controller.serial_action_server_binary:main',
            
        ],
    },
)
