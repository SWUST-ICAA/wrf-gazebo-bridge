from setuptools import setup

package_name = "wrf_gazebo_bridge"


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/wrf_wind_config.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description=(
        "Read WRF netCDF wind fields and publish interpolated wind vectors "
        "at a robot position given by GPS-like topics."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wrf_wind_publisher = wrf_gazebo_bridge.wrf_wind_publisher_node:main",
        ],
    },
)

