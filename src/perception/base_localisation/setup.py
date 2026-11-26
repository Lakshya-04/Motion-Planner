from setuptools import setup

package_name = 'base_localisation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/localisation.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf_diff.yaml', 'config/ekf_omni.yaml',
                                              'config/ekf_ackermann.yaml']),
        ('lib/' + package_name, ['scripts/localisation_check.py', 'scripts/run_localisation_check.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'scan_match_node = base_localisation.scan_match_node:main',
        'sim_imu_node = base_localisation.sim_imu_node:main']},
    maintainer='Lakshya Agarwal',
    maintainer_email='100lakshyaagarwal@gmail.com',
    description='EKF localisation of the base from wheel odometry, IMU and lidar scan matching',
    license='Apache-2.0',
    tests_require=['pytest'],
)
