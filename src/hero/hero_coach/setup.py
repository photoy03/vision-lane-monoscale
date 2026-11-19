from glob import glob

from setuptools import find_packages, setup


package_name = 'hero_coach'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ujin Kwon',
    maintainer_email='ujin@tukorea.ac.kr',
    description='Turns the CARLA + monoscale stack into the hero_hud display contract.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'vehicle_state_adapter = hero_coach.vehicle_state_adapter:main',
            'parking_planner = hero_coach.parking_planner:main',
            'coach_node = hero_coach.coach_node:main',
            'voice_node = hero_coach.voice_node:main',
        ],
    },
)
