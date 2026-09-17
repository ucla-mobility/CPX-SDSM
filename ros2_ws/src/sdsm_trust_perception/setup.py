from glob import glob

from setuptools import find_packages, setup

package_name = 'sdsm_trust_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'filterpy'],
    zip_safe=True,
    maintainer='Lab Maintainer',
    maintainer_email='lab@example.edu',
    description=(
        'Second-layer trust check on SDSM traffic: per-node TrustEngine '
        '(reputation, kinematic-plausibility and size-agreement checks, '
        'corroboration, deferred grace ledger), ported from CPX-Mono '
        'global_trust_perception for CPX-SDSM SensorDataSharingMessage.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'trust_node = sdsm_trust_perception.trust_node:main',
        ],
    },
)
