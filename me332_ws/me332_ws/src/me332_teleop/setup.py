import os
from setuptools import find_packages, setup

package_name = 'me332_teleop'

# 辅助函数：递归收集文件夹中的所有文件，保持目录结构
def package_files(source_directory, target_base):
    paths = []
    for (path, directories, filenames) in os.walk(source_directory):
        for filename in filenames:
            # 源文件路径
            source_file = os.path.join(path, filename)
            # 计算安装后的目标路径 (相对于 site-packages/me332_teleop)
            # 例如: me332_teleop/model/am/final.mdl -> lib/python3.10/site-packages/me332_teleop/model/am/
            rel_path = os.path.relpath(path, source_directory)
            target_dir = os.path.join(target_base, rel_path)
            paths.append((target_dir, [source_file]))
    return paths

# 定义安装的目标路径 (site-packages 路径)
# 注意：这里 hardcode 了 python3.10，如果你的系统升级了 python，需同步修改
install_site_packages = os.path.join('lib', 'python3.10', 'site-packages', package_name)

# 基础数据文件
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # 将 .task 文件放到 site-packages 目录下
    (install_site_packages, [f'{package_name}/hand_landmarker.task']),
]

# 递归增加语音模型文件夹里的所有内容到 site-packages 目录下
data_files += package_files(f'{package_name}/model', os.path.join(install_site_packages, 'model'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # package_data 在 ROS2 环境下有时不稳定，我们主要靠上面的 data_files 确保文件物理拷贝
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liang',
    maintainer_email='liang@todo.todo',
    description='ME332 Teleop package with Keyboard, Voice and Gesture',
    license='TODO: License declaration',
    data_files=data_files,
    entry_points={
        'console_scripts': [
            'teleop_keyboard = me332_teleop.teleop_keyboard:main',
            'teleop_voice = me332_teleop.teleop_voice:main', 
            'teleop_gesture = me332_teleop.teleop_gesture:main',
        ],
    },
)
