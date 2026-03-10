from setuptools import find_packages, setup

package_name = 'mission_dispatcher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Codex',
    maintainer_email='codex@example.com',
    description='Mission dispatcher state machine',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_dispatcher_node = mission_dispatcher.dispatcher_node:main',
            'submit_demo_mission = mission_dispatcher.submit_demo_mission:main',
        ],
    },
)
