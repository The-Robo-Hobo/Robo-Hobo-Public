from setuptools import find_packages, setup

package_name = 'robo_detection'

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
    maintainer='servin',
    maintainer_email='i.am.servinsanchez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # '<executable_name> = <package_name>.<file_name>:main'
            'save_image_node = robo_detection.save_image:main',
            'thermal_publisher_node = robo_detection.thermal_publisher:main',
            'facial_recog_node = robo_detection.facial_recog:main',
            'flash_node = robo_detection.flash_node:main',
            'evidence_recog_node = robo_detection.evidence_recog:main',
        ],
    },
)
