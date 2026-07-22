from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'khj_basic' 

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ("share/" + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob(os.path.join("launch", "*.launch.py"))),
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
            "simple_pub = khj_basic.simple_pub:main",
            "class_pub = khj_basic.class_pub:main",
            "class_sub = khj_basic.class_sub:main",
            "header_pub = khj_basic.header_pub:main",
            "qos_test_pub = khj_basic.qos_test_pub:main",
            "qos_test_sub = khj_basic.qos_test_sub:main"
            "user_int_pub = khj_basic.user_int_pub:main"
        ],
    },
)
