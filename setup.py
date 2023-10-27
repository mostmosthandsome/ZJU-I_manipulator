from glob import glob
from setuptools import setup
from pybind11.setup_helpers import Pybind11Extension

ext_tools = [
    Pybind11Extension(
        "handsome",
        sorted(glob("src/*.cpp")),
        include_dirs=["include"]
    )
]

setup(
    name="HandsomeTools",
    version="0.0.1",
    ext_modules=ext_tools
)