from setuptools import find_packages, setup

package_name = 'oroha_power'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/power.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='OROHA',
    maintainer_email='taesuyim.kopo@gmail.com',
    description='OROHA 전류·전압 계측 프론트엔드 — Raspberry Pi Pico(USB CDC) 브리지',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'power_node = oroha_power.power_node:main',
        ],
    },
)
