from setuptools import find_packages, setup

package_name = 'propeller_plate_iface'

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/propeller_plate_iface.launch.py"]),
        ('share/' + package_name + '/config', ['config/parameters.yaml']),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="roque",
    maintainer_email="roque@caltech.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            'propeller_plate_iface_node = propeller_plate_iface.external_motor_cmd:main',],
    },
)
