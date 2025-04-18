from setuptools import find_packages
from setuptools import setup

setup(
    name='dobot_m1',
    version='0.0.1',
    packages=find_packages(
        include=('dobot_m1', 'dobot_m1.*')),
)
