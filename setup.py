from setuptools import find_packages, setup

package_name = 'map2gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'map2gazebo = map2gazebo.publisher_member_function:main',
        ],
    },
)
