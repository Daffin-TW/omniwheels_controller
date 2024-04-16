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
            'omniwheels_controller_full = omniwheels_kinetic.omniwheels_controller_full:main',
            'omniwheels_controller_cmd_vel = omniwheels_kinetic.omniwheels_controller_cmd_vel:main',
            'robot_service_shoot = omniwheels_kinetic.service_node_test:main'
        ],
    },
)
