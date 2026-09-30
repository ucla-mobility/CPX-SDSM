from glob import glob

from setuptools import find_packages, setup

package_name = 'local_trust_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aron',
    maintainer_email='aronmaung@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_shape = local_trust_estimation.object_shape_node:main',
            'lidar_point_count = local_trust_estimation.lidar_point_count_node:main',
            'object_distance = local_trust_estimation.object_distance_node:main',
            'temporal_presence = '
            'local_trust_estimation.temporal_presence_node:main',
            'trustworthiness_score = '
            'local_trust_estimation.trustworthiness_score_node:main',
            'trustworthiness_visualization = '
            'local_trust_estimation.trustworthiness_visualization_node:main',
            'sdsm_publisher = '
            'local_trust_estimation.sdsm_publisher_node:main',
            'evaluate_offline = local_trust_estimation.evaluate_offline:main',
        ],
    },
)
