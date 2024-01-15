from setuptools import setup,find_packages

setup(
    author="yanbin jin",
    author_email='xxxx@gmail.com',
    python_requires='>=3.6',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3.8',
    ],
    description="HDmap_L2I_py",
    install_requires=['numpy'],
    license="{{ cookiecutter.open_source_license }}",
    include_package_data=True,
    name="HDmap_L2I_py",
    packages=find_packages(),
    test_suite='test',
    version='1.0.0',
    zip_safe=False,
)