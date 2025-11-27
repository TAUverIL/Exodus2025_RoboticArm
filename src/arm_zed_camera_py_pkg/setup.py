from setuptools import find_packages, setup

package_name = 'arm_zed_camera_py_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'pyzed',
        'ultralytics',
        'cv_bridge'
    ],
    zip_safe=True,
    maintainer='arm2025',
    maintainer_email='arm2025@todo.todo',
    description='A ROS 2 package to integrate YOLO with ZED camera for person detection and distance measurement.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # Remove py_modules here
    entry_points={
        'console_scripts': [
            'zed_camera_node = arm_zed_camera_py_pkg.zed_vs:main',
        ],
    },
)
