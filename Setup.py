from setuptools import setup, Extension

# Compile *main.cpp* into a shared library 
setup(
    #...
    ext_modules=[Extension('MemoryManager', ['main.cpp'],extra_compile_args=["-Ofast", "-I./eigen/Eigen"],),],
)
