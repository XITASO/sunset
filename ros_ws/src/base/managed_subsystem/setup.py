from setuptools import find_packages, setup
from glob import glob
import os

package_name = "managed_subsystem"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "resources"), glob("resources/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dockuser",
    maintainer_email="blind@review.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "blackboard_setter = managed_subsystem.blackboard_setter:main",
            "abstract_blackboard_setter = managed_subsystem.abstract_blackboard_setter:main",
            "camera = managed_subsystem.camera:main",
            "depth = managed_subsystem.depth:main",
            "image_enhancement = managed_subsystem.image_enhancement:main",
            "sensor_fusion = managed_subsystem.sensor_fusion:main",
            "segmentation = managed_subsystem.segmentation:main",
        ],
    },
)
