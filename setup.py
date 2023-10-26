from setuptools import find_packages, setup

package_name = 'team19_navigate_to_goal'

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
    maintainer='burger',
    maintainer_email='burger@todo.todo',
    description='Navigates to the given goal while avoiding obstacles',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'goToGoal=team19_navigate_to_goal.goToGoal:main',
                            'TwistControl=team19_navigate_to_goal.TwistControl:main',
        ],
    },
)
