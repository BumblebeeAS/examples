import os
from glob import glob

from setuptools import find_packages, setup


package_name = "bluerov_tasks"


def package_data_files(directory):
    """Install files recursively while preserving their relative directories."""
    return [
        (
            os.path.join("share", package_name, os.path.dirname(path)),
            [path],
        )
        for path in glob(os.path.join(directory, "**", "*"), recursive=True)
        if os.path.isfile(path)
    ]


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        *package_data_files("launch"),
        *package_data_files("config"),
    ],
    scripts=glob("scripts/*.py"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description=(
        "BlueROV2 mission, behavior tree, control, task TF, and vision "
        "launch examples"
    ),
    license="MIT",
)
