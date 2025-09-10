from setuptools import setup, find_packages

setup(
    name="docking_python",  # Name of the overall project
    version="1.0.0",
    description="A project containing docking algorithms and FMU wrappers",
    author="Ronja Kræmer",
    author_email="rkhansen01@gmail.com",
    packages=find_packages(),  # Automatically find all packages in the project
    install_requires=[
        # List the external libraries your project depends on
        'numpy',
        'pythonfmu',
        'hydra-core',
        'omegaconf'
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',  # Minimum Python version required
)
