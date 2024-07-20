from setuptools import find_packages, setup

package_name = 'robitcubebot_python'

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
    maintainer='omar',
    maintainer_email='120633090+omarabelgwad@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleoperation_keyboard= robitcubebot_python.teleop_keyboard:main",
            "nav2_node_custom= robitcubebot_python.nav2_test:main",
            "miniterm= robitcubebot_python.miniterm:main",
            "arduino_transimitter= robitcubebot_python.simple_serial_transmitter:main",
            "arduino_receiver= robitcubebot_python.simple_serial_receiver:main",
            
            

        ],
    },
)
