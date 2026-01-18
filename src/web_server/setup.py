from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'web_server'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include templates
        (os.path.join('share', package_name, 'templates'), glob('templates/*.html')),
        # Include static files if any
        (os.path.join('share', package_name, 'static'), glob('static/*') if os.path.exists('static') else []),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='Ujjwal',
    maintainer_email='ujjwalpatil20@gmail.com',
    description='Flask-based web interface for warehouse control and order management',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server_node = web_server.server_node:main',
        ],
    },
)
