from setuptools import find_packages, setup

package_name = 'whack_a_mole'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/camera.launch.py']),
        ('share/' + package_name, ['config/tags.yaml']),
        ('share/' + package_name, ['config/tags_tf.rviz']),
        ('share/' + package_name, ['config/moveit.rviz']),
        ('share/' + package_name, ['package.xml', 
                                   'launch/planner_swing.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PushkarDave',
    maintainer_email='pushkar@u.northwestern.edu',
    description='Making franka play whack a mole',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swing = whack_a_mole.swing:main',
            'comm_node = whack_a_mole.serial_comm:node_main',
            'camera = whack_a_mole.camera:entry'
            'game_node = whack_a_mole.game:main',
        ],
    },
)
