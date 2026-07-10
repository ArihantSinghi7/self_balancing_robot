from setuptools import find_packages, setup

package_name = 'sbr_controls'

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
    maintainer='arihant-singhi',
    maintainer_email='arihantsinghi@outlook.com',
    description='This package contains the control files for the self balancing robot.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        "state_observer = sbr_controls.state_observer:main",
        "balance_control = sbr_controls.balance_control:main"
        ],
    },
)
