from setuptools import find_packages, setup

package_name = 'gamma_droid_bridge_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
         ["launch/gamma_droid_bridge_launch.py"]),
        ("share/" + package_name + "/config",
         ["config/bridge_config.yaml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student19',
    maintainer_email='student19@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
