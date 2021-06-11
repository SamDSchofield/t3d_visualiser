#!/usr/bin/env python

"""The setup script."""

from setuptools import setup, find_packages

with open('README.md') as readme_file:
    readme = readme_file.read()

requirements = [
    'Click>=7.0',
    "t3d @ git+https://github.com/SamDSchofield/t3d@main#egg=t3d",
    "Pillow",
    "open3d",
    "transforms3d",
    "matplotlib"
]

setup_requirements = ['pytest-runner', ]

test_requirements = ['pytest>=3', ]

setup(
    author="Sam Schofield",
    author_email='sam.schofield@pg.canterbury.ac.nz',
    python_requires='>=3.5',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
    ],
    description="A tool for visualising transforms and po",
    entry_points={
    },
    install_requires=requirements,
    license="MIT license",
    long_description=readme + '\n',
    include_package_data=True,
    keywords='t3d_visualiser',
    name='t3d_visualiser',
    packages=find_packages(include=['t3d_visualiser', 't3d_visualiser.*']),
    setup_requires=setup_requirements,
    test_suite='tests',
    tests_require=test_requirements,
    url='https://github.com/SamDSchofield/t3d_visualiser',
    version='0.1.0',
    zip_safe=False,
)
