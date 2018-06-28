from setuptools import setup

setup(name='CH53',
      version='1.0',
      description='CH-53 Sea Stallion Module',
      url='https://github.com/skilkis/CH-53',
      author='San Kilkis, Nelson Johnson',
      author_email='Kilkis@student.tudelft.nl, n.a.johnson@student.tudelft.nl',
      license='GNU',
      packages=['CH53'],
      install_requires=[
          'numpy',
          'math',
          'matplotlib',
          'scipy',
          'control',
      ],
      zip_safe=False)
