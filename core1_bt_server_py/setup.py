from setuptools import find_packages, setup

package_name = 'core1_bt_server_py'

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
    maintainer='taiga',
    maintainer_email='ray255ar@gmail.com',
    description='TODO: Package description',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'example_node = core1_bt_server_py.example_node:main',
            'tar_server = core1_bt_server_py.tar_server:main',
            'tar_client = example_client.tar_client:main',
        ],
    },
)