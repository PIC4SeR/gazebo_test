from setuptools import setup

package_name = "gazebo_test_dashboard"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    package_data={package_name: ["example_campaign.yaml"]},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", package_name + "/example_campaign.yaml"]),
    ],
    install_requires=["setuptools", "flask", "pandas", "pyyaml"],
    zip_safe=True,
    maintainer="Andrea Ostuni",
    maintainer_email="andrea.ostuni@polito.it",
    description="Local web dashboard that runs gazebo_test campaigns as a queue of batches over parallel lanes.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Serve the campaign control page (launches ros2 gazeboexp run per batch).
            "gazebo_test_dashboard = gazebo_test_dashboard.server:main",
        ],
    },
)
