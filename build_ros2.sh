REPO_ROOT=`pwd`

# Download local ASIO
# git clone https://github.com/chriskohlhoff/asio

cd $REPO_ROOT
mkdir src
# wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < $REPO_ROOT/ros2.repos

cd $REPO_ROOT
colcon build --symlink-install --packages-skip-by-dep python_qt_binding \
    --cmake-args -DTHIRDPARTY=FORCE # -DAsio_INCLUDE_DIR=$REPO_ROOT/asio/asio/include/
