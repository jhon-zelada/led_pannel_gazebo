from setuptools import find_packages, setup
import os 
from glob import glob 

package_name = 'led_panel_sim'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (f'share/{package_name}/models/led_panel', ['models/led_panel/model.config', 'models/led_panel/model.sdf']),
        (f'share/{package_name}/models/led_panel/meshes', glob('models/led_panel/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhon',
    maintainer_email='jhonzelada01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'QR_publisher_executable = led_panel_sim.QR_publisher:main',
            'code_publisher_executable = led_panel_sim.code_publisher:main',
            
        ],
    },
)
