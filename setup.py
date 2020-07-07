from setuptools import setup


with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name="JetsonEV",
    version="0.0.1",
    author="Peter Somers",
    author_email="peter.somers@isys.uni-stuttgart.de",
    description="A Python module to control the ISYS EVs",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.tik.uni-stuttgart.de/ac121730/JetsonEV",
    packages=['JetsonEV', 'JetsonEV/utilities'],
    package_data={'JetsonEV/utilities': ['JetsonEV/utilities/imu_calibration.txt']},
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
    ],
    python_requires='>=3',
    install_requires=['TimedTask',
                      'PyDataSocket',
                      'NanoCamera',
                      'ArduinoI2C',
                      'FastestRplidar',
                      'xbox360controller',
                      'mpu9x50']
)
