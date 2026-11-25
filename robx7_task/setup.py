import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robx7_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='angel',
    maintainer_email='angelignacio7991@gmail.com',
    description='Gestor de tarea del robx7',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager = robx7_task.task_manager:main',
        ],
    },
)
