from setuptools import find_packages, setup

package_name = 'mission_operator_hmi'

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
    description='Unified operator HMI for mission, mobility, and excavation planning.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'integrated_operator_hmi = mission_operator_hmi.integrated_operator_hmi:main',
        ],
    },
)
