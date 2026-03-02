from setuptools import find_packages, setup

package_name = "pm_robot_dashboard"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/app.launch.py"]),
        ('share/' + package_name, ['pm_robot_dashboard/documentation/images/app_icon.png']),
        ('share/' + package_name, ['pm_robot_dashboard/documentation/images/match_Logo_cut.png']),
        ('share/' + package_name, ['pm_robot_dashboard/documentation/images/match_Logo.png']),

    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pmlab",
    maintainer_email="pmlab",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["pm_robot_dashboard = pm_robot_dashboard.pm_robot_dashboard:main"],
    },
)
