from setuptools import setup, Extension

controller_core_module = Extension(
    "bng_controller.core.controller_core",
    sources=["controller_core.c"],
    extra_compile_args=["-O3"],
)

setup(
    name="bng_controller_core",
    version="0.1",
    ext_modules=[controller_core_module],
    description="High-level controller core computations in C",
)
