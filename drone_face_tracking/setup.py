import os
from glob import glob
from setuptools import setup

package_name = 'drone_face_tracking'

def get_data_files(directory, destination):
    """Recursively collect data files from a directory."""
    data_files = []
    for dirpath, dirnames, filenames in os.walk(directory):
        if not filenames:
            continue
        dest = os.path.join(destination, os.path.relpath(dirpath, directory))
        src  = [os.path.join(dirpath, f) for f in filenames]
        data_files.append((dest, src))
    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Package index marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Gazebo world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        # MAVROS / general config
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Models — recurse into subdirectories
        *get_data_files('models', os.path.join('share', package_name, 'models')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Face tracking drone companion computer package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = drone_face_tracking.camera_node:main',
            'face_detect_node = drone_face_tracking.face_detect_node:main',
            'flight_ctrl_node = drone_face_tracking.flight_ctrl_node:main',
            'mission_node = drone_face_tracking.mission_node:main',
            'plot_errors_node = drone_face_tracking.plot_errors_node:main',
        ],
    },
)