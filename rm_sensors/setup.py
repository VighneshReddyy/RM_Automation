from setuptools import find_packages, setup

package_name = 'rm_sensors'

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
    maintainer='exhausted',
    maintainer_email='shirs.saisarath@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu=rm_sensors.imu_l1_l2:main',
            'emu_3=rm_sensors.imu_bevel_also:main',
        ],
    },
)
