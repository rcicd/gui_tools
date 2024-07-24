from setuptools import find_packages, setup
import os
from glob import glob


def map_recursive_files(dest, directory):
    return os.path.join(dest, directory), [y for x in os.walk(directory) for y in glob(os.path.join(x[0], '*'))]


package_name = 'intrinsic_calibration_tool'
packages = ['calibration_gui']

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    package_dir={'': 'include'},

    data_files=[
                   ('share/ament_index/resource_index/packages',
                    ['resource/' + package_name]),
                   ('share/' + package_name, ['package.xml']),
                   ('share/' + package_name + '/assets', glob('assets/*.jpg')),
                   ('share/' + package_name + '/launch', glob('launch/*.launch')),
                   ('share/' + package_name + '/src', glob('src/*.py')),
                   ('lib/' + package_name, glob('src/*.py'))
               ]
               + [map_recursive_files('share/' + package_name, 'include/' + x) for x in os.listdir('include')],
               # + [map_recursive_files('share/' + package_name, 'config/' + x) for x in os.listdir('config')],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ilia.Nechaev',
    maintainer_email='lelikk2002@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "intrinsic_calibration_node = intrinsic_calibration_tool.intrinsic_calibration_node:main"
        ],
    },
)
