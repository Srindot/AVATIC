from glob import glob

from setuptools import find_packages, setup

PACKAGE = 'pluto_x_demo'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{PACKAGE}']),
        (f'share/{PACKAGE}', ['package.xml']),
        (f'share/{PACKAGE}/config', glob('config/*.yaml')),
        (f'share/{PACKAGE}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srindot',
    maintainer_email='srinath.bhamidipati@research.iiit.ac.in',
    description='AVATIC workshop demo: planned-route balloon popping.',
    license='TBD',
    tests_require=['pytest'],
)
