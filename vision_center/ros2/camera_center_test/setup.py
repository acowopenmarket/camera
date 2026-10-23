from setuptools import setup
setup(name='camera_center_test', version='0.1.0', packages=['camera_center_test'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/camera_center_test']),
                  ('share/camera_center_test', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['center_point = camera_center_test.center_point:main']})
