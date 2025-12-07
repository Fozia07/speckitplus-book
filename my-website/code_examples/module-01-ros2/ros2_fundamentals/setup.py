from setuptools import setup

package_name = 'ros2_fundamentals'

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
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Examples for Module 1: ROS 2 Fundamentals',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = ros2_fundamentals.simple_publisher:main',
            'simple_subscriber = ros2_fundamentals.simple_subscriber:main',
            'simple_service_server = ros2_fundamentals.simple_service_server:main',
            'simple_service_client = ros2_fundamentals.simple_service_client:main',
            'simple_action_server = ros2_fundamentals.simple_action_server:main',
            'simple_action_client = ros2_fundamentals.simple_action_client:main',
        ],
    },
)
