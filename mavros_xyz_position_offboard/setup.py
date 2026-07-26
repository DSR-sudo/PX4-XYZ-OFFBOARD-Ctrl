from setuptools import setup


package_name = "mavros_xyz_position_offboard"

setup(
    name=package_name,
    version="0.1.0",
    py_modules=[
        "artifact_log",
        "cli",
        "core",
        "core_types",
        "node_callbacks",
        "node_flight",
        "node_services",
        "node_status",
        "planner",
        "range_guard",
        "ros2_position_node",
        "ros_app",
        "safety_checks",
        "safety_core",
    ],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Safety-gated ROS 2 MAVROS native XYZ position monitor and controller.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mavros_xyz_position_node = ros2_position_node:main",
        ],
    },
)
