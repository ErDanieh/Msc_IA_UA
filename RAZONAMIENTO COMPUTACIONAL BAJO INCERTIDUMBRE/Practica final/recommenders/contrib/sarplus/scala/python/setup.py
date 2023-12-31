# Copyright (c) Recommenders contributors.
# Licensed under the MIT License.

from distutils.core import setup
from pathlib import Path

setup(
    name="pysarplus_dummy",
    version=(Path(__file__).resolve().parent.parent.parent / "VERSION")
    .read_text()
    .strip(),
    description="pysarplus dummy package to trigger spark packaging",
    author="RecoDev Team at Microsoft",
    author_email="recodevteam@service.microsoft.com",
    url="https://github.com/Microsoft/Recommenders/contrib/sarplus",
    packages=["pysarplus_dummy"],
)
