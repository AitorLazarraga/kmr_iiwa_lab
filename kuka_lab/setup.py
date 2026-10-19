from setuptools import find_packages, setup
import os

package_name = 'kuka_lab'


def package_files(directory):
    """Devuelve [(destino, [ficheros])] conservando las subcarpetas."""
    paths = []
    for root, _, files in os.walk(directory):
        if files:
            paths.append((
                os.path.join('share', package_name, root),
                [os.path.join(root, f) for f in files],
            ))
    return paths


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    + package_files('launch')
    + package_files('urdf')
    + package_files('world')
    + package_files('config')
    + package_files('meshes')
    + package_files('rviz'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='idlab',
    maintainer_email='idlab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
