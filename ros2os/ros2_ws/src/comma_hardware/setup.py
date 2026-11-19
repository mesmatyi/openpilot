from setuptools import find_packages, setup

package_name = 'comma_hardware'

setup(
  name=package_name,
  version='0.1.0',
  packages=find_packages(exclude=['test']),
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='ros2os maintainers',
  maintainer_email='noreply@example.com',
  description='Device health, thermal and power management for comma devices',
  license='MIT',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      'hardware_node = comma_hardware.node:main',
    ],
  },
)
