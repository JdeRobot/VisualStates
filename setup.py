from distutils.core import setup

setup(
    version='...',
    scripts=['src/visualStates.py'],
    packages=['gui', 'codegen', 'codegen.python'],
    package_dir={'':'src'}
)
