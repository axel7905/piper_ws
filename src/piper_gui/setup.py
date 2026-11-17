from setuptools import setup
from glob import glob
setup(name='piper_gui', version='0.1.0', packages=['piper_gui'],
 data_files=[('share/ament_index/resource_index/packages',['resource/piper_gui']),
 ('share/piper_gui',['package.xml']),('share/piper_gui/launch',glob('launch/*.py')),
 ('share/piper_gui/config',glob('config/*.yaml'))],
 install_requires=['setuptools'], zip_safe=True, maintainer='Axel',
 maintainer_email='axel@example.com', description='PiPER course joint control',license='MIT',
 entry_points={'console_scripts':['gui_node = piper_gui.gui_node:main']})
