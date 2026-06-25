from setuptools import find_packages, setup

package_name = 'offboard_test'

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
    maintainer='clear',
    maintainer_email='chasequarkko@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'offboard_node = offboard_test.offboard_node:main',
'nmpc_node = offboard_test.nmpc_node:main',
'plot_logger = offboard_test.plot_logger:main',
'metrics_collector = offboard_test.metrics_collector:main',
        ],
    },
)
