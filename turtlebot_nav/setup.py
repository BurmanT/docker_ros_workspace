from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install scripts
        (os.path.join('lib', package_name),
            glob('src/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TurtleBot navigation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'random_navigator = turtlebot_nav.random_navigator:main',
            'goal_extractor   = turtlebot_nav.goal_extractor:main',
        ],
    },
)
