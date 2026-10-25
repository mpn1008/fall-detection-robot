from setuptools import find_packages, setup


setup(
    name="turtlebot3_motion",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/turtlebot3_motion"]),
        ("share/turtlebot3_motion", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mpn1008",
    maintainer_email="mpn1008@users.noreply.github.com",
    description="A bounded, one-shot TurtleBot3 velocity test for Isaac Sim.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["move_a_bit = turtlebot3_motion.move_a_bit:main"],
    },
)
