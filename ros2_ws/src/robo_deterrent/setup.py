from setuptools import find_packages, setup

package_name = 'robo_deterrent'

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
    maintainer='jiro',
    maintainer_email='jirooblea@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # '<executable_name> = <package_name>.<file_name>:main'
            'siren_node = robo_deterrent.siren:main',
        ],
    },
)
