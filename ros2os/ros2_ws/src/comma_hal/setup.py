from setuptools import find_packages, setup

package_name = 'comma_hal'

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
  description='Hardware abstraction helpers for comma devices',
  license='MIT',
  tests_require=['pytest'],
)
