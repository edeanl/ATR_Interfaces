import os
from glob import glob
from setuptools import setup


package_name = 'atr_examples_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dean',
    maintainer_email='dean@chalmers.se',
    description='Python ATR interface tutorial ros2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_pubsubs_class = atr_examples_py.minimal_pubsubs_class',
            'minimal_publisher = atr_examples_py.minimal_publisher:main',
            'minimal_subscriber = atr_examples_py.minimal_subscriber:main',
            'dummy_node = atr_examples_py.applications.dummy:main'
        ],
    },
)
