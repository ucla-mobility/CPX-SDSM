from setuptools import find_packages, setup

package_name = "veins_ros_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lab Maintainer",
    maintainer_email="lab@example.edu",
    description="UDP bridge Veins <-> ROS 2; uses sdsm_msgs (CPX-SDSM).",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "udp_bridge_node = veins_ros_bridge.udp_bridge_node:main",
        ],
    },
)
