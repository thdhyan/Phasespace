from glob import glob

from setuptools import setup

package_name = "phasespace_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/trackers", glob("trackers/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dhyan Thakkar",
    maintainer_email="thakk100@umn.edu",
    description="PhaseSpace OWL client for ROS 2",
    license="MIT",
    entry_points={"console_scripts": ["tracker = phasespace_ros2.tracker:main"]},
)
