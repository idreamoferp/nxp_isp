import setuptools

with open('README.md') as readme_file:
    README = readme_file.read()

setup_args = dict(
    name='nxpprog',
    version='0.2.8',
    pymodules=['nxpprog'],
    description='Programmer for NXP arm processors using ISP protocol',
    long_description_content_type='text/markdown',
    long_description=README,
    license='MIT',
    entry_points={
        'console_scripts': ['nxpprog=nxpprog:main'],
    },
    author='SJSU-Dev2 Organization',
    author_email='kammcecorp@gmail.com',
    keywords=['NXP', 'SJSU', 'SJSU-Dev2'],
    url='https://github.com/SJSU-Dev2/nxpprog',
    download_url='https://pypi.org/project/nxpprog/'
)

install_requires = [
    'intelhex',
    'pyserial',
    'Click'
]

if __name__ == '__main__':
    setuptools.setup(**setup_args, install_requires=install_requires)
