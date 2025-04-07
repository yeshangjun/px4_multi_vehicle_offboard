import os
from glob import glob
from setuptools import setup

package_name = 'px4_multi_plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*.[yma]*'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools', 'pytest'],
    zip_safe=True,
    maintainer='Ye Shangjun',
    maintainer_email='yeshangjun@zju.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                f'offboard_control = {package_name}.offboard_control:main',
                # 'visualizer = px4_single_plan.visualizer:main',
                f'velocity_control_1 = {package_name}.velocity_control_1:main',
                f'velocity_control_2 = {package_name}.velocity_control_2:main',
                f'velocity_control_3 = {package_name}.velocity_control_3:main',
                f'control = {package_name}.control:main',
                f'processes = {package_name}.processes:main',
                f'simulation_node = {package_name}.simulation_node:main'
        ],
    },
)
