#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup


setup(
    name='claptrap_simu',
    version='0.0.1',
    description='Simulation and log processing for Claptrap',
    long_description=open('README.md').read(),
    packages=['claptrap_simu'],
    package_dir={'': 'src'},
    entry_points = {
        'console_scripts': ['claptrap_plotter=claptrap_simu.log_handling.plotter:main',
                            'claptrap_replay=claptrap_simu.log_handling.viewer3D:main'],
    },
    data_files=[('urdf', ['data/claptrap.urdf', 'data/wheel.dae', 'data/body.dae'])],
    zip_safe=False)
