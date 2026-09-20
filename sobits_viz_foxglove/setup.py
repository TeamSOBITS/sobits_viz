from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'sobits_viz_foxglove'


def config_files() -> list:
    """List config/ keeping its directory structure, as data_files entries."""
    entries = []
    for root, dirs, files in os.walk('config'):
        dirs[:] = [d for d in dirs if d != '__pycache__']
        wanted = [f for f in files if not f.endswith('.pyc')]
        if wanted:
            target = os.path.join('share', package_name, root)
            entries.append((target, [os.path.join(root, f) for f in wanted]))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        *config_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VALENTIN Keith',
    maintainer_email='kvalentincardenas@gmail.com',
    description="Serve a TeamSOBITS robot's cameras, TF, joints and model to the Foxglove viewer",
    license='BSD-3-Clause',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'description_relay = sobits_viz_foxglove.description_relay:main',
            'make_layout = sobits_viz_foxglove.make_layout:main',
            'transform_throttle = sobits_viz_foxglove.transform_throttle:main',
        ],
    },
)
