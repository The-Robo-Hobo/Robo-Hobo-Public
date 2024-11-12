from setuptools import find_packages, setup

package_name = 'aros2duino'

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
            "serial_comm = aros2duino.serial_comm:main",
            "find_ports = aros2duino.find_ports:main",
            "test_writing = aros2duino.test_writing:main"
        ],
    },
)