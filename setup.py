import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'map2gazebo'

# Helper function to recursively gather all files in a directory
def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'models', 'map', 'meshes'), package_files(os.path.join('models', 'map', 'meshes'))),
        (os.path.join('share', package_name, 'models', 'map'), ['models/map/model.config', 'models/map/model.sdf']),
        """
        Add more models here, look at the example above or below for the format.
        """
        # (os.path.join('share', package_name, 'models', 'unisee', 'meshes'), package_files(os.path.join('models', 'unisee', 'meshes'))),
        # (os.path.join('share', package_name, 'models', 'unisee'), ['models/unisee/model.config', 'models/unisee/model.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raunak',
    maintainer_email='raunak@uni-bremen.de',
    description='The map2gazebo package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map2gazebo = map2gazebo.map2gazebo:main',
        ],
    },
)
