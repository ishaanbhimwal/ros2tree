from setuptools import find_packages, setup

package_name = "tree"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "ros2cli"],
    zip_safe=True,
    maintainer="Ishaan Bhimwal",
    maintainer_email="ishaanbhimwal@protonmail.com",
    description="ROS 2 CLI tool to view node/topic tree, inspired by the tree command.",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "ros2cli.command": [
            "tree = tree.tree:TreeCommand",
        ],
        "tree.verb": [
            "nodes = tree.tree:NodesVerb",
            "topics = tree.tree:TopicsVerb",
        ],
    },
)
