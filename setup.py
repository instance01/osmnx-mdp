from setuptools import setup
from setuptools import find_packages


setup(
   name='osmnx_mdp',
   version='1.0',
   description='',
   author='instance01',
   packages=find_packages(),
   install_requires=['osmnx', 'networkx', 'numpy'],
)
