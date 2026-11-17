from setuptools import find_packages, setup

setup(
    name="pi_arm_tasks",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/pi_arm_tasks"]),
        ("share/pi_arm_tasks", ["package.xml", "README.md"]),
        ("share/pi_arm_tasks/zero_reference", ["pi_arm_tasks/zero_reference/README.md"]),
        ("share/pi_arm_tasks/calibration_motion", ["pi_arm_tasks/calibration_motion/README.md"]),
        ("share/pi_arm_tasks/examples", ["examples/calibration_poses.json"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Pi Arm Maintainer",
    maintainer_email="maintainer@example.com",
    description="Reusable tasks using Pi Arm ROS services and actions.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "pi_arm_execution = pi_arm_tasks.execution.node:main",
        "pi_arm_zero_reference = pi_arm_tasks.zero_reference.node:main",
        "pi_arm_zero = pi_arm_tasks.zero_reference.cli:main",
        "pi_arm_zero_once = pi_arm_tasks.zero_reference.standalone:main",
        "pi_arm_calibration_motion = pi_arm_tasks.calibration_motion.cli:main",
    ]},
)
