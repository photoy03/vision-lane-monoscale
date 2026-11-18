from glob import glob

from setuptools import find_packages, setup


package_name = "parking_slot_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tukorea32",
    maintainer_email="tukorea32@example.com",
    description="Persistent parking-slot mapping and goal selection",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "slot_map_node = parking_slot_mapping.slot_map_node:main",
        "goal_selector_node = parking_slot_mapping.goal_selector_node:main",
    ]},
)
