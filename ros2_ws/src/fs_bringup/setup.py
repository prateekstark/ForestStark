from glob import glob

from setuptools import find_packages, setup

setup(
    name="fs_bringup",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[("share/ament_index/resource_index/packages", ["resource/fs_bringup"]),
                ("share/fs_bringup", ["package.xml"]),
                ("share/fs_bringup/launch", glob("launch/*.launch.py")),
                ("share/fs_bringup/rviz", glob("rviz/*.rviz"))],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ForestStark",
    maintainer_email="garg.prateek@alumni.iitd.ac.in",
    description="ForestStark simulation launch files",
    license="Proprietary",
)
