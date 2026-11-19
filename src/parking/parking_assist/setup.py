from glob import glob

from setuptools import setup

package_name = "parking_assist"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jaehun",
    description="Parking goal pose from parking-line segments",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "goal_pose_node = parking_assist.goal_pose_node:main",
        "goal_selector_node = parking_assist.goal_selector_node:main",
        # The owner's detector (parking_space_detector.py), included unchanged.
        "parking_space_detector = parking_assist.parking_space_detector:main",
        # The same detector fed from the camera's JPEG topic (--jpeg-topic), for the vehicle.
        "parking_space_detector_jpeg = parking_assist.parking_space_detector_jpeg:main",
    ]},
)
