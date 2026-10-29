from glob import glob

from setuptools import find_packages, setup


setup(
    name='px4_bridge',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/px4_bridge']),
        ('share/px4_bridge', ['package.xml', 'README.md']),
        ('share/px4_bridge/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SPR framework/VIO team',
    maintainer_email='maintainers@example.invalid',
    description='Read-only PX4 SensorCombined to FLU IMU bridge.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['px4_bridge = px4_bridge.node:main']},
)
