from setuptools import find_packages, setup

NODES = ["localization", "gnss_degrader", "planting_head", "katar_driver", "katar_planter", "jaal_fleet_manager",
         "jaal_navigator", "livestock", "pahadi_navigator", "stability_monitor", "traction_monitor", "mission_logger",
         "viz_markers", "safety_monitor", "pedestrians"]

setup(
    name="fs_autonomy",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[("share/ament_index/resource_index/packages", ["resource/fs_autonomy"]),
                ("share/fs_autonomy", ["package.xml"])],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="ForestStark",
    maintainer_email="garg.prateek@alumni.iitd.ac.in",
    description="ForestStark planting robot autonomy",
    license="Proprietary",
    entry_points={"console_scripts": [f"{n} = fs_autonomy.nodes.{n}:main" for n in NODES]
                  + ["operator = fs_autonomy.nodes.operator_tablet:main"]},
)
