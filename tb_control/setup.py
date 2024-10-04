from setuptools import find_packages, setup

package_name = 'tb_control'

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
    maintainer='neeraj',
    maintainer_email='neeraj@todo.todo',
    description='Open Loop Controller for TurtleBot3',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb_openLoop = tb_control.tb_openLoop:main',
            'tb_scenario1 = tb_control.tb_scenario1:main',
            'tb_scenario2 = tb_control.tb_scenario2:main',
            'tb_pose_plotter = tb_control.tb_pose_plotter:main',
        ],
    },
)
