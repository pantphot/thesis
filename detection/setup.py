from setuptools import find_packages
from setuptools import setup

package_name = 'detection'

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
    author='Pantelis Photiou',
    author_email='pantphotiou@gmail.com',
    maintainer='Pantelis Photiou',
    maintainer_email='pantphotiou@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Package containing a node that transforms from two frames.',
    license='Apache License, Version 2.0',
    test_suite='test',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_publisher = detection.scripts.target_publisher.py:main',
        ],
    },
)
