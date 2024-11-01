from setuptools import setup, find_packages

setup(
    name='robot_swarm_bridge',
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        'signalrcore>=0.9.5',
        'pyyaml>=6.0.1',
        'python-dotenv>=1.0.0'
    ],
    author='Your Name',
    author_email='your.email@example.com',
    description='Bridge between ROS and .NET backend for robot swarm control',
    license='MIT',
    python_requires='>=3.8',
)