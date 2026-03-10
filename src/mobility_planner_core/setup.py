from setuptools import find_packages, setup

package_name = 'mobility_planner_core'

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
    description='Walk action adapter',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobility_action_server = mobility_planner_core.action_server:main',
            'mock_nav2_server = mobility_planner_core.mock_nav2_server:main',
        ],
    },
)
