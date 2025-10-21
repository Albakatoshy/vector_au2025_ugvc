from setuptools import find_packages, setup

package_name = 'victor_planning'

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
    maintainer='abood',
    maintainer_email='anaalbakatoshy.gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "a_star_planner = victor_planning.a_star_planner:main",
            "OccupancyGrid_pub = victor_planning.OccupancyGrid_pub:main",
            "way_point_navigator = victor_planning.way_point_navigator:main",
            "simple_curve_path_publisher = victor_planning.simple_curve_path_publisher:main",
            "rviz_goal_curve_path_publisher = victor_planning.rviz_goal_curve_path_publisher:main",


        ],
    },
)
