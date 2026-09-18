from setuptools import find_packages, setup

package_name = 'global_trust_tracker'

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
    maintainer='Alex Markova',
    maintainer_email='alinka.markova@gmail.com',
    description='SORT tracker node for the V2X trust pipeline.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'tracker = global_trust_tracker.tracker_node:main',
        ],
    },
)
