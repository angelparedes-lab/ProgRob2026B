import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robx7_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='angel',
    maintainer_email='angelignacio7991@gmail.com',
    description='Nodos de percepcion del robx7',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_viewer = robx7_vision.camera_viewer:main',
            'object_detector = robx7_vision.object_detector:main',
        ],
    },
)
