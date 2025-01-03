from setuptools import setup
import os
from glob import glob

package_name = "rexasi_tracker"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.[pxy][yma]*')))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marco",
    maintainer_email="marco.piazzola@spindox.it",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
