from setuptools import find_packages, setup

package_name = 'bumperbot_py_examples'

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
    maintainer='jebear',
    maintainer_email='jerepondumie@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlesim_kinematics = bumperbot_py_examples.turtlesim_kinematics:main',
            'simple_tf_kinematics = bumperbot_py_examples.simple_tf_kinematics:main',
        ],
    },
)
