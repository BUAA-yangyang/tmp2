from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=["a1_result_manager"],
    package_dir={"": "src"},
)

setup(**setup_args)
