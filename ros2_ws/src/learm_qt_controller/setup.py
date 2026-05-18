from setuptools import find_packages, setup

package_name = 'learm_qt_controller'

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
    maintainer='ubunturos',
    maintainer_email='ubunturos@todo.todo',
    description='QT5 interface to control LeArm via MoveIt2',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'learm_qt_controller = learm_qt_controller.run_qt_controller:main',
        ],
    },
)
