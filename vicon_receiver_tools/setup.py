"""Setup of the vicon_receiver_tools ROS 2 package (ament_python)."""

from setuptools import find_packages, setup

PACKAGE_NAME = "vicon_receiver_tools"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Marker file registering the package in the ament index
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PeterC",
    maintainer_email="petercalifano.gs@gmail.com",
    description="Python tools of the vicon_receiver repository (config access, diagnostics)",
    license="GPL-3.0",
)
