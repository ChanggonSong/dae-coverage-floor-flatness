from setuptools import setup, find_packages
import glob
import os

package_name = 'dae_coverage_floor_flatness'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        # Config files(params.yaml 등) 관리
        # 참고로 Input/Output files(맵, 경로 파일 등)는 사용자 홈 디렉토리의 외부 저장소(~/dae_floor_maps)에서 관리하므로 패키지에는 포함하지 않음
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
        # rviz files
        (os.path.join('share', package_name, 'rviz'), glob.glob('rviz/*.rviz')),
        # urdf files
        (os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*')),
        # world files
        (os.path.join('share', package_name, 'worlds'), glob.glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Changgon',
    maintainer_email='gon05158557@gmail.com',
    description='DAE 기반 실내 환경 모델링과 커버리지 주행을 이용한 3D LiDAR 바닥 평탄도 자율 측정 시스템',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_executor = mission_execution.mission_executor:main',
            'surface_profiler = surface_profiling.surface_profiler:main',
        ],
    },
)
