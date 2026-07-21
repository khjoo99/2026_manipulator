from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'khj_example_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khjoo',
    maintainer_email='khjoo990408@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'm_pub = khj_example_1.m_pub:main',
            'm_sub = khj_example_1.m_sub:main',
            'm2_sub = khj_example_1.m2_sub:main',
            't_pub = khj_example_1.t_pub:main',
            'mt_sub = khj_example_1.mt_sub:main',
            'mv_turtle = khj_example_1.mv_turtle:main',
        ],
    },
)
