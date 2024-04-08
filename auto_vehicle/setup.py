from setuptools import find_packages, setup

package_name = 'auto_vehicle'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Schaff',
    maintainer_email='mscha16@lsu.edu',
    description='An autonomous vehicle controller that can handle stop signs, road following, and intersections.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam = auto_vehicle.webcam:main',
            'vision = auto_vehicle.vision:main',
            'planner = auto_vehicle.planner:main',
            'hardware = auto_vehicle.hardware:main',
            'localization = auto_vehicle.localization:main'
        ],
    },
)
