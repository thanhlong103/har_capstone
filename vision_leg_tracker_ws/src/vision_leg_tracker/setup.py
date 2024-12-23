import os
from glob import glob 
from setuptools import setup

package_name = 'vision_leg_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=[
                    'setuptools', 
                    'tensorflow'
    ],
    zip_safe=True,
    maintainer='ndphu',
    maintainer_email='ndphu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_leg_tracker = vision_leg_tracker.vision_leg_tracker:main',
            'test = vision_leg_tracker.test:main'
        ],
    },
)
