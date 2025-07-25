from setuptools import find_packages, setup

package_name = 'ros2-pure-pursuit'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Dekhterman',
    maintainer_email='sdekhterman@gmail.com',
    description='A driving controller that tracks a set distance ahead of the vehicle.',
    license='GNU',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pur    = ros2-pure-pursuit.pure_pur:main',
            'track_log   = ros2-pure-pursuit.track_log:main',
            'path_visual = ros2-pure-pursuit.path_visual:main'
        ],
    },
)
