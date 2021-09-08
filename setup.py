#!/usr/bin/env python
"""
.. codeauthor:: Gavin Suddreys
"""
import os
from typing import List
from setuptools import setup, find_packages

here = os.path.abspath(os.path.dirname(__file__))

req = [
    'numpy>=1.18.0',
    'spatialmath-python>=0.8.7',
    'roboticstoolbox-python>=0.8',
    'pyyaml'
]

dev_req = [
    'pytest',
    'pytest-cov',
    'flake8',
    'sympy'
]

docs_req = [
    'sphinx',
    'sphinx_rtd_theme',
    'sphinx-autorun',
]

# Get the long description from the README file
with open(os.path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

# list all data folders here, to ensure they get packaged

data_folders = [
]


def package_files(directory: List[str]) -> List[str]:
    """[summary]

    :param directory: [description]
    :type directory: [type]
    :return: [description]
    :rtype: [type]
    """
    paths: List[str] = []
    for (pathhere, _, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', pathhere, filename))
    return paths


extra_files = []
for data_folder in data_folders:
    extra_files += package_files(data_folder)

setup(
    name='armer',

    version='0.1.0',

    description='Armer - The ROS Arm drivEr',

    long_description=long_description,

    long_description_content_type='text/markdown',

    url='https://github.com/suddrey-qut/armer',

    author='Gavin Suddrey',

    license='MIT',

    classifiers=[
        #   3 - Alpha
        #   4 - Beta
        #   5 - Production/Stable
        'Development Status :: 3 - Alpha',

        # Indicate who your project is intended for
        'Intended Audience :: Developers',
        # Pick your license as you wish (should match "license" above)
        'License :: OSI Approved :: MIT License',

        # Specify the Python versions you support here. In particular, ensure
        # that you indicate whether you support Python 2, Python 3 or both.
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],

    python_requires='>=3.6',

    project_urls={
        # 'Documentation': 'https://petercorke.github.io/roboticstoolbox-python',
        'Source': 'https://github.com/suddrey-qut/armer',
        'Tracker': 'https://github.com/suddrey-qut/armer/issues'#,
        # 'Coverage': 'https://codecov.io/gh/petercorke/roboticstoolbox-python'
    },

    keywords='python robotics robotics-toolbox kinematics dynamics' \
             ' motion-planning trajectory-generation jacobian hessian' \
             ' control simulation robot-manipulator mobile-robot ros',

    packages=find_packages(exclude=['tests']),
    package_data={'armer': extra_files},

    include_package_data=True,

    install_requires=req,

    extras_require={
        'dev': dev_req,
        'docs': docs_req
    }
)
