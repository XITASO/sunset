from setuptools import find_packages, setup

package_name = 'python_base_class'

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
    maintainer='Blind',
    maintainer_email="blind@review.com",
    description='ENGEL base class',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_class = python_base_class.base_class:main'
        ],
    },
)
