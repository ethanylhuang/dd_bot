import os
from glob import glob
from setuptools import setup

package_name = 'dd_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs your launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # ADD THIS LINE to install the URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
        # ADD THIS LINE to install the config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)