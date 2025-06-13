from setuptools import setup

package_name = 'tello_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/circle_tracker.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luke Concini',
    maintainer_email='lconcini@ualberta.ca',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_tracker_node = tello_cv.circle_tracker_node:main',
            'pid_controller_node = tello_cv.pid_controller_node:main'
        ],
    },
)