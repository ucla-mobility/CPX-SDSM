from glob import glob

from setuptools import find_packages, setup

package_name = 'global_trust_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/assets',
            glob(package_name + '/assets/*.glb')
            + glob(package_name + '/assets/*.urdf')
            # The clipped lanelet map is gitignored (*.osm) but still installed
            # locally so lanelet_overlay resolves it like the meshes; the 121 MB
            # raw source lives in /.osm_convert/, NOT assets/, so it is not
            # globbed here. See tools/lichtblick/clip_lanelet_osm.py.
            + glob(package_name + '/assets/*.osm')),
        ('share/' + package_name + '/launch',
            glob('launch/*launch.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools', 'numpy', 'scipy', 'shapely', 'filterpy'],
    zip_safe=True,
    maintainer='Alex Markova',
    maintainer_email='alinka.markova@gmail.com',
    description='Trustworthy cooperative perception: reputation-based trust scoring for V2X agents using SORT tracking, MS-PSF rotated-BEV fusion, and dynamic thresholding.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'agent = global_trust_perception.pipeline.agent:main',
            'scene = global_trust_perception.lichtblick.scene_node:main',
            'trust_view = global_trust_perception.lichtblick.trust_view_node:main',
            'ground_backdrop = global_trust_perception.lichtblick.ground_backdrop_node:main',
            'lanelet_overlay = global_trust_perception.lichtblick.lanelet_overlay_node:main',
        ],
    },
)
