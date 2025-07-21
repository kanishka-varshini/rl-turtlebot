from setuptools import setup

package_name = 'ttb4_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Hybrid ROS 2 package with C++ sim and Python viz',
    license='MIT',
    entry_points={},
)
