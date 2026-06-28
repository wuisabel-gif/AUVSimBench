import os
from glob import glob
from pathlib import Path

from setuptools import setup

package_name = "auv_sim_bench"

setup(
    name=package_name,
    version="0.1.1",
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
    keywords=[
        "ros2",
        "auv",
        "underwater-robotics",
        "marine-robotics",
        "robotics",
        "simulation",
        "physics-simulation",
        "digital-twin",
        "state-estimation",
        "localization",
        "autonomous-underwater-vehicle",
    ],
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Physics",
        "Framework :: Robot Framework",
    ],
    entry_points={
        "console_scripts": [
            "sim = auv_sim_bench.sim_node:main",
        ],
    },
)
