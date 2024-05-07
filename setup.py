from setuptools import setup, find_packages

package_name = 'multiple_topic_monitor'

setup(
    name=package_name,
    version='1.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yukihiro Saito',
    maintainer_email='yukky.saito@gmail.com',
    description='ROS 2 package for monitoring the frequency and delay of multiple topics.',
    license='Apache License 2.0; BSD-3-Clause',
    tests_require=[
        'pytest',
        'ament_copyright',
        'ament_flake8',
        'ament_pep257',
        'python3-pytest'
    ],
    entry_points={
        'console_scripts': [
            'multiple_topic_monitor = multiple_topic_monitor.multiple_topic_monitor:main'
        ],
    },
)
