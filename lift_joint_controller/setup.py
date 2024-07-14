from setuptools import find_packages, setup

package_name = 'lift_joint_controller'

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
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package to control lift joint',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lift_joint_publisher = lift_joint_controller.lift_joint_publisher:main'
        ],
    },
)
