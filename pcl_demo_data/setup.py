from setuptools import find_packages, setup

package_name = 'pcl_demo_data'

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
    maintainer='ninad',
    maintainer_email='nsa.ninad@gmail.com',
    description='Synthetic PointCloud2 publisher for testing the ROS 2 PCL perception pipeline',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'synthetic_pointcloud_publisher = pcl_demo_data.synthetic_pointcloud_publisher:main',
        ],
    },
)
