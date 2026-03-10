from setuptools import find_packages, setup

package_name = 'excavation_planner_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'std_srvs'],
    zip_safe=True,
    maintainer='Codex',
    maintainer_email='codex@example.com',
    description='Dig action adapter',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'excavation_action_server = excavation_planner_core.action_server:main',
            'mock_pointcloud_server = excavation_planner_core.mock_pointcloud_server:main',
            'legacy_perception_notifier = excavation_planner_core.legacy_perception_notifier:main',
            'legacy_dig_planner_orchestrator = excavation_planner_core.legacy_dig_planner_orchestrator:main',
        ],
    },
)
