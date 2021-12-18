from setuptools import setup, find_packages

setup(
    name='nxp_isp', 
    version='1.0', 
    packages=find_packages(),
    install_requires=['pyserial', 'intelhex==2.1']
    )