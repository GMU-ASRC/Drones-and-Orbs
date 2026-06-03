from setuptools import setup

package_name = 'flight_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.search', package_name + '.tests'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Pre-built flight test missions for the Hexsoon 450',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_runner = flight_tests.flight_test:main',
        ],
    },
)
