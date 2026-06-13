# from setuptools import find_packages
from setuptools import setup

package_name = "tools_demo"
modules = "tools_demo/modules"
setup(
    name=package_name,
    version="0.0.0",
    # packages=find_packages(exclude=["test"]),
    packages=[package_name, modules],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sunrise",
    maintainer_email="sunrise@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # "tf_pub = tools_demo.tf_pub:main",
            "pose_get = tools_demo.pose_get:main",
            "chat = tools_demo.chat_interactive:main",
            "connecter = tools_demo.connecter:main",
            "test = tools_demo.test:main",
            "langgraph_agent = tools_demo.modules.agent.agent_node:main",
        ],
    },
)
