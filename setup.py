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
            "cpp_mdp",
            ["osmnx_mdp/algorithms/cpp_mdp.cpp", "osmnx_mdp/serialize_util.cpp"],
            include_dirs=["osmnx_mdp/external/"]
        ),
        Extension(
            "cpp_dstar_lite",
            ["osmnx_mdp/algorithms/cpp_dstar_lite.cpp"],
            include_dirs=["osmnx_mdp/external/"]
        ),
        Extension(
            "cpp_improved_dstar_lite",
            ["osmnx_mdp/algorithms/cpp_improved_dstar_lite.cpp"],
            include_dirs=["osmnx_mdp/external/"]
        ),
        Extension(
            "cpp_brtdp",
            ["osmnx_mdp/algorithms/cpp_brtdp.cpp", "osmnx_mdp/serialize_util.cpp"],
            include_dirs=["osmnx_mdp/external/"]
        ),
        Extension(
            "cpp_brtdp_replan",
            ["osmnx_mdp/algorithms/cpp_brtdp_replan.cpp"],
            include_dirs=["osmnx_mdp/external/"]
        ),
        Extension(
            "lib",
            ["osmnx_mdp/cpp_lib.cpp"],
            libraries=["m"]
        ),
        "osmnx_mdp/algorithms/*.pyx",
        "osmnx_mdp/*.pyx",
        "osmnx_mdp/debug/*.pyx",
        "osmnx_mdp/tests/*.pyx",
    ],
        language="c++",
        include_path=["osmnx_mdp/external/"]
    )
)
