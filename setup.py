from setuptools import setup

package_name = 'ros_3d_attractor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a.whit',
    maintainer_email='nml@whit.contact',
    description='A ROS2 package that implements a node that generates force commands suitable for attracting a robotic effector to a point, line, or plane embedded in a 3D space',
    license='Mozilla Public License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = ros_3d_attractor.entry_point:main'
        ],
    },
)
