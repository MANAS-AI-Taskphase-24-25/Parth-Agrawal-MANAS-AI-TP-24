from setuptools import find_packages, setup

package_name = 'drone_lawnmower'

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
    maintainer='parth_234',
    maintainer_email='personal.parth.agr.234@outlook.com',
    description='ROS2 package for drone lawnmower mission with DroneKit and point cloud visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_lawnmower = drone_lawnmower.drone_lawnmower:main',
        ],
    },
)
