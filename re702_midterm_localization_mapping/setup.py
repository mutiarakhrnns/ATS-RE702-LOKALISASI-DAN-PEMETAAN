from setuptools import find_packages, setup

package_name = 're702_midterm_localization_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='syahputra',
    maintainer_email='dannisyahputra36@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation = re702_midterm_localization_mapping.navigation:main',
            'test_audio = re702_midterm_localization_mapping.test_audio:main',
        ],
    },
)
