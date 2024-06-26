from setuptools import find_packages, setup

package_name = 'task_allocation'

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
    maintainer_email='omar.adventure22@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_loc = task_allocation.get_loc:main',
            'single_task_allocation_node = task_allocation.single_task_allocation_server:main',
            'multi_task_allocation_node = task_allocation.multi_task_allocation_server:main',
        ],
    },
)
