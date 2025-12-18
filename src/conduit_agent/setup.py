from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'conduit_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Engineer',
    maintainer_email='me@example.com',
    description='Prototype Universal Driver for FactoryOS Orchestration',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'factory_robot = conduit_agent.robot_node:main',
            'orchestrator = conduit_agent.orchestrator_node:main',
        ],
    },
)
