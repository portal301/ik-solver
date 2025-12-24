"""
IKFast Python module setup script
Builds ikfast_solver Python extension module using pybind11
"""

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import sys
import os

class get_pybind_include:
    """Helper class to determine the pybind11 include path"""
    def __str__(self):
        import pybind11
        return pybind11.get_include()

# 프로젝트 경로
project_root = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(project_root, 'src')
include_dir = os.path.join(project_root, 'include')

# 소스 파일
sources = [
    os.path.join(src_dir, 'ikfast_pybind.cpp'),
    os.path.join(src_dir, 'ikfast_core.cpp'),
]

# Extension 모듈 정의
ext_modules = [
    Extension(
        'ikfast_solver',
        sources=sources,
        include_dirs=[
            get_pybind_include(),
            include_dir,
        ],
        language='c++',
        extra_compile_args=['/std:c++17', '/EHsc', '/MD', '/O2', '/utf-8', '/D_CRT_SECURE_NO_WARNINGS'] if sys.platform == 'win32' else ['-std=c++17'],
        define_macros=[('IKFAST_HAS_LIBRARY', None)],
    ),
]

setup(
    name='ikfast_solver',
    version='1.0.0',
    author='IKFast Generator',
    description='IKFast multi-robot inverse kinematics solver',
    long_description='Python bindings for IKFast robot IK solvers',
    ext_modules=ext_modules,
    zip_safe=False,
    python_requires='>=3.7',
    install_requires=[
        'pybind11>=2.6.0',
        'numpy>=1.19.0',
    ],
)
