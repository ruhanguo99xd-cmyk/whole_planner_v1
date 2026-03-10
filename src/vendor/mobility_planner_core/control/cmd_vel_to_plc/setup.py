from setuptools import find_packages, setup

package_name = 'cmd_vel_to_plc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'python-snap7'],
    zip_safe=True,
    maintainer='ruhanguo',
    maintainer_email='ruhanguo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 格式： '执行名 = 包名.文件名:入口函数'
            'cmd_vel_to_plc = cmd_vel_to_plc.cmd_vel_to_plc:main',
        ],
    },
)
