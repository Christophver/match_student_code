from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multi_robot_optimizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Hier sagen wir ROS 2, dass die config und launch Ordner mitkopiert werden sollen:
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Christoph Verhage',
    maintainer_email='student@todo.todo',
    description='PSO Algorithmus für Multi-Roboter Formationen (Masterarbeit)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Hier definieren wir den Startbefehl für den ROS-Knoten (kommt im nächsten Schritt)
            'optimizer_node = multi_robot_optimizer.optimizer_node:main',
            'image_to_ros = multi_robot_optimizer.image_to_ros:main',
        ],
    },
)
