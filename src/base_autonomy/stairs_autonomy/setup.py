from setuptools import setup

package_name = 'stairs_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='all',
    maintainer_email='guofei@cmu.edu',
    description='Stairs-aware autonomy supervisor and motion executor for multi-level navigation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stair_detector_lidar_imu = stairs_autonomy.stair_detector_lidar_imu:main',
            'stair_supervisor = stairs_autonomy.stair_supervisor:main',
            'motion_executor = stairs_autonomy.motion_executor:main',
        ],
    },
)