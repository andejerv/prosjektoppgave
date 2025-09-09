import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="vessel_model_tuning_lib",
    version="0.0.1",
    author="Zeabuz",
    author_email="dev@zeabuz.com",
    description="Vessel model tuning library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=setuptools.find_packages(),
    install_requires=[
        'numpy',
        'scipy',
        'pandas',
        'matplotlib',
        'openplaning'
    ]
)
