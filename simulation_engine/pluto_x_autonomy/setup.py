from glob import glob

from setuptools import find_packages, setup

PACKAGE = 'pluto_x_autonomy'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{PACKAGE}']),
        (f'share/{PACKAGE}', ['package.xml']),
        (f'share/{PACKAGE}/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='srindot',
    maintainer_email='srinath.bhamidipati@research.iiit.ac.in',
    description='Developer outer-loop interface and host for Pluto X simulator validation.',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'outer_loop_host = {PACKAGE}.host_node:main',
        ],
    },
)
