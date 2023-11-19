from setuptools import setup

package_name = 'tutorial_rclpy_package'

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
    maintainer='Kyuhyong',
    maintainer_email='Kyuhyong@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rclpy_pub_sub = tutorial_rclpy_package.rclpy_pub_sub:main',
            'rclpy_service_client = tutorial_rclpy_package.rclpy_service_client:main'
        ],
    },
)
