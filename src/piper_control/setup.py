from setuptools import setup
from glob import glob
setup(name='piper_control', version='0.1.0', packages=['piper_control'],
 data_files=[('share/ament_index/resource_index/packages',['resource/piper_control']),
 ('share/piper_control',['package.xml']),('share/piper_control/launch',glob('launch/*.py')),
 ('share/piper_control/config',glob('config/*.yaml'))],
 install_requires=['setuptools'], zip_safe=True, maintainer='Axel',
 maintainer_email='axel@example.com', description='PiPER course joint control',license='MIT',
 entry_points={'console_scripts':['control_node = piper_control.control_node:main']})
