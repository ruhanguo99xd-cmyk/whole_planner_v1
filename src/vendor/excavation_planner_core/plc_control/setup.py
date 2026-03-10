from setuptools import find_packages, setup

package_name = 'plc_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'std_srvs', 'numpy'],
    zip_safe=True,
    maintainer='pipixuan',
    maintainer_email='1531554489@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 格式："终端命令名 = 包名.文件名:入口函数"
            'plc_control_test = plc_control.plc_control_test:main',
            'plc_control_test1 = plc_control.plc_control_test1:main',
            'plc_control_test2 = plc_control.plc_control_test2:main',
        ],
    },
)
