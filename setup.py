import os
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "auv_sim_bench"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wuisabel-gif",
    maintainer_email="231155141+wuisabel-gif@users.noreply.github.com",
    description="Lightweight 6-DOF AUV physics sim for dry-testing state estimation.",
    long_description=Path("README.md").read_text(encoding="utf-8"),
    long_description_content_type="text/markdown",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sim = auv_sim_bench.sim_node:main",
        ],
    },
)
