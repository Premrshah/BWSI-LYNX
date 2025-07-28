from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_processors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacobyia',
    maintainer_email='jacoby.adwin@gmail.com',
    description='Vision processing nodes, subscribe to cam feeds and output data.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'ar_detector = vision_processors.ar_detector:main',
        'line_detector = vision_processors.line_detector:main',
        'line_controller = vision_processors.line_controller:main',
    ],
    },
)
