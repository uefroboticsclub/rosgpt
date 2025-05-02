# from distutils.core import setup

# if __name__ == "__main__":
#     setup()


from setuptools import setup, find_packages

package_name = 'rosgpt'

setup(
    name=package_name,
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
    author='ROSGPT Team',
    author_email='example@example.com',
    description='ROS Guide Powered by Transformers',
    license='MIT',
    entry_points={
        'console_scripts': [
            'turtle_agent = turtle_agent.scripts.turtle_agent:main',
            'pincherx_agent = pincherx_agent.scripts.pincherx_agent:main',
        ],
    },
)