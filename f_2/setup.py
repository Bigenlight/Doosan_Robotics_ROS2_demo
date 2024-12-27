from setuptools import find_packages, setup

package_name = "f_2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="juwan",
    maintainer_email="dlacksdn352@gmail.com",
    description="ROKEY BOOT CAMP Package",
    license="Apache 2.0 License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pillar_sort=f_2.pillar_sort:main",
            "grip_and_mesure=f_2.grip_and_mesure:main",
            "moving=f_2.moving:main",
            "test=f_2.test:main",
            "pillar_5=f_2.pillar_5:main",
            "pillar_sort_kwi=f_2.pillar_sort_kwi:main",
            "gear=f_2.gear:main",
        ],
    },
)
