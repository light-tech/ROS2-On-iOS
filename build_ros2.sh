REPO_ROOT=`pwd`

python3 -m pip install -U \
 argcomplete catkin_pkg colcon-common-extensions coverage \
 cryptography empy flake8 flake8-blind-except==0.1.1 flake8-builtins \
 flake8-class-newline flake8-comprehensions flake8-deprecated \
 flake8-docstrings flake8-import-order flake8-quotes \
 importlib-metadata lark==1.1.1 lxml matplotlib mock mypy==0.931 netifaces \
 nose pep8 psutil pydocstyle pydot pygraphviz pyparsing==2.4.7 \
 pytest-mock rosdep rosdistro setuptools==59.6.0 vcstool

mkdir -p ros2_humble/src
cd ros2_humble
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos

cd $REPO_ROOT/ros2_humble/
colcon build --symlink-install --packages-skip-by-dep python_qt_binding
