from setuptools import setup, find_packages

package_name = 'turtle_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/agent.launch']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='goldenglorys',
    maintainer_email='olusolagloryolamide@gmail.com',
    description='ROSGPT agent for the TurtleSim robot.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_agent = turtle_agent.turtle_agent:main',
        ],
    },
)