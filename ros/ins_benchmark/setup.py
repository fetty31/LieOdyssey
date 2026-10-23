from setuptools import find_packages, setup

package_name = 'ins_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vaive',
    maintainer_email='fetty3113@gmail.com',
    description='ROS 2 Humble benchmark framework for replaying data through ins_ros.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ins_benchmark = ins_benchmark.cli:main',
            'ins_benchmark_node = ins_benchmark.benchmark_node:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ins_benchmark.launch.py']),
        ('share/' + package_name + '/config', ['config/benchmark_config.yaml']),
    ],
)
