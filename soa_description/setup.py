from setuptools import find_packages, setup

package_name = 'soa_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', [
            'urdf/soa_macro.xacro',
            'urdf/soa_ros2_control.xacro',
            'urdf/soa.urdf.xacro',
            'urdf/dual_soa.urdf.xacro',
        ]),
        ('share/' + package_name + '/meshes', [
            'meshes/base.stl',
            'meshes/base_motor_holder.stl',
            'meshes/motor_holder_base.stl',
            'meshes/motor_holder_wrist.stl',
            'meshes/moving_jaw.stl',
            'meshes/rotation_pitch.stl',
            'meshes/sts3215_03a.stl',
            'meshes/sts3215_03a_no_horn.stl',
            'meshes/under_arm.stl',
            'meshes/upper_arm.stl',
            'meshes/waveshare_mounting_plate.stl',
            'meshes/wrist_roll_follower.stl',
            'meshes/wrist_roll_pitch.stl',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='42076119+htchr@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
