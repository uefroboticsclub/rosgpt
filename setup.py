from setuptools import setup, find_packages

setup(
    name='rosgpt',
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        'python-dotenv',
        'langchain',
        'langchain-groq',
        'pyinputplus',
        'rich',
        'regex',
    ],
)