from setuptools import find_packages, setup
from glob import glob
import os

package_name = "experiment_setup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "resources"), glob("resources/*.yaml")),
        (os.path.join("share", package_name, "resources", "scenarios"), glob("resources/scenarios/*.yaml")),
        (os.path.join("share", package_name, "resources", "scenarios", "fixedScenarios"), glob("resources/scenarios/fixedScenarios/*.yaml")),
        (os.path.join("share", package_name, "templates"), glob("experiment_setup/templates/*.jinja")),
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
            "evaluator = experiment_setup.evaluator:main",
            "scenario_executor = experiment_setup.scenario_executor:main",
            "experiment_logger = experiment_setup.experiment_logger:main",
            "setup_file_generator = experiment_setup.experiment_setup.setup_file_generator:main",
        ],
    },
)
