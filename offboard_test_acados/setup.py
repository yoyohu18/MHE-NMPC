import glob

from setuptools import find_packages, setup

package_name = 'offboard_test_acados'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob.glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clear',
    maintainer_email='chasequarkko@gmail.com',
    description='acados-based NMPC port of offboard_test/nmpc_node, kept isolated for comparison',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'acados_nmpc_node = offboard_test_acados.acados_nmpc_node:main',
            'plot_logger_acados = offboard_test_acados.plot_logger_acados:main',
            'mhe_node = offboard_test_acados.mhe_node:main',
            'drone_tf_broadcaster = offboard_test_acados.drone_tf_broadcaster:main',
            'prop_joint_state_publisher = offboard_test_acados.prop_joint_state_publisher:main',
            'proximity_gripper_node = offboard_test_acados.proximity_gripper_node:main',
            'gripper_flight_node = offboard_test_acados.gripper_flight_node:main',
        ],
    },
)
