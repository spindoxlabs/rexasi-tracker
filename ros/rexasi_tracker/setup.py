from setuptools import setup, find_packages
import os
from glob import glob

package_name = "rexasi_tracker"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marco",
    maintainer_email="marco.piazzola@spindox.it",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
          'tracker = rexasi_tracker.tracker:main',
          'track_fusion = rexasi_tracker.track_fusion:main'
        ],
    },
)
