from setuptools import setup
from glob import glob

package_name = 'tang_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('urdf/*.urdf') ),
        ('share/' + package_name, glob('mesh/*.obj') ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nonroot',
    maintainer_email='nonroot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
