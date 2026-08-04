from setuptools import find_packages, setup

package_name = "camera_opencv"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
        "console_scripts": 
            ["img_show = camera_opencv.img_show:main",
            "img_write = camera_opencv.img_write:main",
            "img_pub = camera_opencv.img_pub:main",
            "img_compressed_pub = camera_opencv.img_compressed_pub:main",
            "img_sub = camera_opencv.img_sub:main",
            "img_compressed_sub = camera_opencv.img_compressed_sub:main",
            "camera_pub = camera_opencv.camera_pub:main",
            "camera_sub = camera_opencv.camera_sub:main",
            "circle_follow = camera_opencv.circle_follow:main",
            "circle_follow_teacher = camera_opencv.circle_follow_teacher:main",
            
            ],
        
    },
)
