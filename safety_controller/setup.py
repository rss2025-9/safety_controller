from setuptools import find_packages, setup

package_name = 'safety_controller'

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
    maintainer='racecar',
    maintainer_email='selinna@mit.edu',
    description='Safety Controller for our racecar',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_controller = safety_controller.safety_controller:main',
        ],
    },
)
