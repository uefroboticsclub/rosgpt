# from setuptools import setup, find_packages

# package_name = 'turtle_agent'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/' + package_name, ['package.xml']), 
#         ('share/' + package_name + '/launch', ['launch/agent.launch']), 
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='goldenglorys',
#     maintainer_email='olusolagloryolamide@gmail.com',
#     description='ROSGPT agent for the TurtleSim robot.',
#     license='Apache 2.0',
#     entry_points={
#         'console_scripts': [
#             'turtle_agent = turtle_agent.turtle_agent:main',
#         ],
#     },
# )

from setuptools import setup, find_packages

package_name = 'turtle_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.'), 
    package_dir={'': '.'}, 
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/agent.launch']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='ROSGPT agent for the TurtleSim robot.',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'turtle_agent = turtle_agent.turtle_agent:main', 
        ],
    },
)