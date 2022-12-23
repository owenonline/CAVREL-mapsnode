from setuptools import setup

package_name = 'py_gpsnode'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='owenb',
    maintainer_email='owenburns88@gmail.com',
    description='publishes the closest coordinate to the position of the car',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_gpsnode.publisher_member_function:main',
            'demo = py_gpsnode.demo_node:main'
        ],
    },
)
