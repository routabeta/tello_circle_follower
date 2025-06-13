from setuptools import setup

package_name = 'tello_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='lconcini@ualberta.ca',
    description='Teleop keyboard for Tello Boost Combo Drone',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard_node = tello_control.teleop_keyboard:main'
        ],
    },
)
