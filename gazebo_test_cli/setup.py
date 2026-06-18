from setuptools import setup

package_name = "gazebo_test_cli"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, package_name + ".command"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "argcomplete", "libtmux"],
    zip_safe=True,
    maintainer="Andrea Ostuni",
    maintainer_email="andrea.ostuni@polito.it",
    description="CLI front-end for gazebo_test social-navigation experiments.",
    license="MIT",
    tests_require=["pytest"],
    # NOTE: intentionally NO setup.cfg script_dir/install_scripts redirect to
    # lib/<pkg>. That keeps the console script in <prefix>/bin (on PATH), which
    # is what makes `register-python-argcomplete full_experiment` work.
    entry_points={
        "console_scripts": [
            "full_experiment = gazebo_test_cli.full_experiment:main",
        ],
        # Native `ros2 gazeboexp ...` command (completion via ros2cli's argcomplete).
        "ros2cli.command": [
            "gazeboexp = gazebo_test_cli.command.gazeboexp:GazeboExpCommand",
        ],
    },
)
