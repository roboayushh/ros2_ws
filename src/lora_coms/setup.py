from setuptools import find_packages, setup

package_name = 'lora_coms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pyserial','sx127x'],
    zip_safe=True,
    maintainer='nextauv',
    maintainer_email='nextauv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lora_node = lora_coms.lora_node:main'
        ],
    },
)
