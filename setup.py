# -*- coding: utf-8 -*-

from setuptools import setup, find_packages


with open('README.md') as f:
    readme = f.read()

# with open('LICENSE') as f:
#     license = f.read()

setup(
    name='iksolver',
    version='0.0.1',
    description='An implementation of robotic iksolver.',
    long_description=readme,
    author='Puttichai, Pham Tien Hung',
    author_email='abc',
    url='https://github.com/Puttichai/iksolver',
    # license=license,
    packages=find_packages(exclude=('tests'))
)

