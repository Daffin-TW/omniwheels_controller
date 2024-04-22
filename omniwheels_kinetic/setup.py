from setuptools import find_packages, setup

package_name = 'omniwheels_kinetic'

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
    maintainer='daffin_tw',
    maintainer_email='daffin_tw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_to_cmd_vel = omniwheels_kinetic.joystick_to_cmd_vel:main',
            'go_to_goal = omniwheels_kinetic.go_to_goal:main',
            'uwb = omniwheels_kinetic.uwb:main',
            'robot_service_shoot = omniwheels_kinetic.service_node_test:main'
        ],
    },
)
