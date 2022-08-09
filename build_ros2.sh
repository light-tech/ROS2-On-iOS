RREPO_ROOT=`pwd`

python3 -m pip install --user -U \
 argcomplete catkin_pkg colcon-common-extensions coverage \
 cryptography empy flake8 flake8-blind-except==0.1.1 flake8-builtins \
 flake8-class-newline flake8-comprehensions flake8-deprecated \
 flake8-docstrings flake8-import-order flake8-quotes \
 importlib-metadata lark==1.1.1 lxml matplotlib mock mypy==0.931 netifaces \
 nose pep8 psutil pydocstyle pydot pyparsing==2.4.7 \
 pytest-mock rosdep rosdistro setuptools==59.6.0 vcstool #pygraphviz

# Temporarily use the GitHub Action user script installation location
export PATH=$PATH:/Users/runner/Library/Python/3.9/bin

# Download local ASIO
# git clone https://github.com/chriskohlhoff/asio

cd $REPO_ROOT
mkdir -p src
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos

cd $REPO_ROOT
colcon build --symlink-install --packages-skip-by-dep python_qt_binding \
    --cmake-args -DTHIRDPARTY=FORCE # -DAsio_INCLUDE_DIR=$REPO_ROOT/asio/asio/include/
