from setuptools import setup
from setuptools import find_packages
from Cython.Build import cythonize
from Cython.Distutils import build_ext
from distutils.extension import Extension


setup(
    name='osmnx_mdp',
    version='1.0',
    description='',
    author='Instance01',
    packages=find_packages(),
    install_requires=['osmnx', 'networkx', 'numpy'],
    include_dirs=["osmnx_mdp/"],
    cmdclass={"build_ext": build_ext},
    ext_modules=cythonize([
        Extension(
            "testthis",
            ["osmnx_mdp/algorithms/testthis.cpp"],
            include_dirs=["osmnx_mdp/algorithms/"]
        ),
        Extension(
            "cpp_dstar_lite",
            ["osmnx_mdp/algorithms/cpp_dstar_lite.cpp"],
            include_dirs=["osmnx_mdp/algorithms/"]
        ),
        Extension(
            "lib",
            ["osmnx_mdp/lib.cpp"],
            libraries=["m"]
        ),
        "osmnx_mdp/algorithms/*.pyx",
        "osmnx_mdp/*.pyx",
        "osmnx_mdp/tests/*.pyx",
    ],
        language="c++",
        include_path=["osmnx_mdp/algorithms/"]
    )
)
