from setuptools import find_packages, setup

package_name = 'qos_relay'

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
    maintainer='arpan',
    maintainer_email='arpanekka.a1.73@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # --- ADD THIS ENTIRE SECTION ---
    entry_points={
        'console_scripts': [
            'scan_relay_node = qos_relay.scan_relay_node:main',
        ],
    },
    # ------------------------------
)