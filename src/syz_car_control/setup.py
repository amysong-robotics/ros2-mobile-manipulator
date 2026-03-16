from setuptools import find_packages, setup

package_name = 'syz_car_control'

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
    maintainer='syz',
    maintainer_email='syz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gesture_control = syz_car_control.gesture_control:main',
            'simple_control = syz_car_control.simple_control:main',
            'test = syz_car_control.test:main',
        ],
    },
)
