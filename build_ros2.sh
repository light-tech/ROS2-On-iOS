REPO_ROOT=`pwd`

# Download local ASIO
# git clone https://github.com/chriskohlhoff/asio

cd $REPO_ROOT
mkdir src
# wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < $REPO_ROOT/ros2_min.repos

cd $REPO_ROOT
colcon build --symlink-install --packages-skip-by-dep python_qt_binding --cmake-args -DTHIRDPARTY=FORCE -DFORCE_BUILD_VENDOR_PKG=ON -DBUILD_SHARED_LIBS=NO -DBUILD_TESTING=NO # --executor sequential --event-handlers console_direct+ -DAsio_INCLUDE_DIR=$REPO_ROOT/asio/asio/include/
