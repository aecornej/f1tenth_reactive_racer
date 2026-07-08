from setuptools import find_packages, setup

package_name = 'f1tenth_reactive_racer'

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
    maintainer='andrew',
    maintainer_email='andrew@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'reactive_node = f1tenth_reactive_racer.reactive_follower:main',
        'opp_reactive_node = f1tenth_reactive_racer.opp_reactive_follower:main',
        'lap_timer_node = f1tenth_reactive_racer.lap_timer:main'
        ],
    },
)
