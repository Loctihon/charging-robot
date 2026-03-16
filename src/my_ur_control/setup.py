from setuptools import find_packages, setup

package_name = 'my_ur_control'

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
    maintainer='loc',
    maintainer_email='loc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'donghocnghich = my_ur_control.arm_commander:main',
            # 'donghocnghich = my_ur_control.only_arm_commander:main',
            'donghocnghich = my_ur_control.donghocnghich:main',
            'donghocthuan = my_ur_control.DH_Thuan:main',

        ],
    },
)
