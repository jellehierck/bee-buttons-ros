from setuptools import find_packages, setup

package_name = 'bee_buttons'

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
    maintainer='Jelle Hierck',
    maintainer_email='j.j.hierck@student.utwente.nl',
    description='ROS node to interface with the Bee Buttons',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bee_buttons = bee_buttons.bee_buttons:main'
        ],
    },
)
