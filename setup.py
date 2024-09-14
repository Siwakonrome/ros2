import os
from glob import glob
from setuptools import setup, find_packages



PACKAGE_NAME = 'lecture3_noise_generator'

with open("requirements.txt", "r", encoding="utf-8-sig") as f:
    requirements = [i.strip() for i in f.readlines()]

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', PACKAGE_NAME, 'resource'), glob('resource/*')),
        (os.path.join('share', PACKAGE_NAME, 'config'), glob('config/*.yaml')),
        (os.path.join('share/ament_index/resource_index/packages'), [os.path.join('resource', PACKAGE_NAME)]),
    ],
    install_requires=requirements,
    zip_safe=True,
    maintainer='Siwakon Rommueang',
    maintainer_email='tiger.prasang@gmail.com',
    description='Description of your ROS 2 package',
    license='License type, e.g., Apache 2.0 or MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'noise_generator = {PACKAGE_NAME}.noise_generator:main',
            f'velocity_mux = {PACKAGE_NAME}.velocity_mux:main',
        ],
    },
)
