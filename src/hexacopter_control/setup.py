from setuptools import find_packages, setup

package_name = 'hexacopter_control'

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
    maintainer='pranjal',
    maintainer_email='workpran48@gmail.com',
    description='Waypoint follower with tracking-error logging for the X4 hexacopter',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['wp=hexacopter_control.wp:main'
        ],
    },
)
