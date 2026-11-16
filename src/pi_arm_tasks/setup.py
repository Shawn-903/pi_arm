from setuptools import find_packages, setup

setup(
    name="pi_arm_tasks",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/pi_arm_tasks"]),
        ("share/pi_arm_tasks", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Pi Arm Maintainer",
    maintainer_email="maintainer@example.com",
    description="Reusable tasks using Pi Arm ROS services and actions.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "pi_arm_tasks = pi_arm_tasks.node:main",
        "pi_arm_zero = pi_arm_tasks.cli:main",
    ]},
)
