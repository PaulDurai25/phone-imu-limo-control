from setuptools import setup

package_name = 'phone_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agilex',
    maintainer_email='agilex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'phone_imu_node = phone_imu.phone_imu_node:main',
	'phone_to_limo = phone_imu.phone_to_limo:main',
	'phone_yaw_test = phone_imu.phone_yaw_test:main',
	'phone_orientation_dryrun = phone_imu.phone_orientation_dryrun:main',
	'phone_orientation_controller = phone_imu.phone_orientation_controller:main',
        ],
    },
)
