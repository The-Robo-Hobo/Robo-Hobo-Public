from setuptools import find_packages, setup

package_name = 'robo_navigation'

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
    maintainer='serv',
    maintainer_email='i.am.servinsanchez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # '<executable_name> = <package_name>.<file_name>:main'
            'arduino_comms_node = robo_navigation.arduino_comms_node:main'
        ],
    },
)
