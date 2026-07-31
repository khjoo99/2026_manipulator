import os
from glob import glob

from setuptools import find_packages, setup

package_name = "tf2_basic"


def collect_data_files(source_directory, install_directory):
    collected_files = []

    for root, directories, files in os.walk(source_directory):
        if not files:
            continue

        relative_path = os.path.relpath(root, source_directory)

        if relative_path == ".":
            destination = install_directory
        else:
            destination = os.path.join(
                install_directory,
                relative_path,
            )

        source_files = [
            os.path.join(root, filename)
            for filename in files
        ]

        collected_files.append(
            (destination, source_files)
        )

    return collected_files


data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name],
    ),
    (
        "share/" + package_name,
        ["package.xml"],
    ),
    (
        "share/" + package_name + "/launch",
        glob(os.path.join("launch", "*.launch.py")),
    ),
    (
        "share/" + package_name + "/urdf",
        glob(os.path.join("urdf", "*.*")),
    ),
    (
        "share/" + package_name + "/rviz",
        glob(os.path.join("rviz", "*.*")),
    ),
    (
        "share/" + package_name + "/meshes",
        glob(os.path.join("meshes", "*.*")),
    ),
    (
        "share/" + package_name + "/world",
        glob(os.path.join("world", "*.*")),
    ),
    (
        "share/" + package_name + "/data",
        glob(os.path.join("data", "*.yaml")),
    ),
]

data_files += collect_data_files(
    "models",
    os.path.join("share", package_name, "models"),
)


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="khjoo",
    maintainer_email="khjoo990408@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "static_turtle_tf2_broadcaster = tf2_basic.static_turtle_tf2_broadcaster:main",
            "dynamic_turtle_tf2_broadcaster = tf2_basic.dynamic_turtle_tf2_broadcaster:main",
            "tf_listener = tf2_basic.tf_listener:main",
            "turtle_tf_listener = tf2_basic.turtle_tf_listener:main",
            "follow_turtle = tf2_basic.follow_turtle:main",
            "move_u2d2 = tf2_basic.move_u2d2:main",
            "move_manipulator = tf2_basic.move_manipulator:main",
            "dance_manipulator = tf2_basic.dance_manipulator:main",
            "move_manipulator_action = tf2_basic.move_manipulator_action:main",
            "move_manipulator_action_temp = tf2_basic.move_manipulator_action_temp:main",
            "dance_manipulator_action = tf2_basic.dance_manipulator_action:main",
            "teach_manipulator = tf2_basic.teach_manipulator:main",
            "teach_manipulator_t1 = tf2_basic.teach_manipulator_t1:main",
            "play_recorded_dance = tf2_basic.play_recorded_dance:main",
            "record_pick_place = tf2_basic.record_pick_place:main",
            "play_recorded_pick_place = tf2_basic.play_recorded_pick_place:main",
            "moveit_test = tf2_basic.moveit_test:main",
            "moveit_class = tf2_basic.moveit_class:main",
            "moveit_scene_monitor = tf2_basic.moveit_scene_monitor:main",
            "moveit_attached = tf2_basic.moveit_attached:main",
            "moveit_mini_project = tf2_basic.moveit_mini_project:main",
        ],
    },
)