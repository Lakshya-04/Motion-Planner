from setuptools import setup

package_name = 'sim_realism'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/realism.launch.py']),
        ('lib/' + package_name, ['scripts/run_realism_check.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'odom_realism_node = sim_realism.odom_realism_node:main',
        'depth_realism_node = sim_realism.depth_realism_node:main']},
    maintainer='Lakshya Agarwal',
    maintainer_email='100lakshyaagarwal@gmail.com',
    description='Odometry and depth noise for the simulators',
    license='Apache-2.0',
    tests_require=['pytest'],
)
