# Build ROS2 Base + Rviz for macOS
# Assuming dependencies are built prior

REPO_ROOT=`pwd`

cd $REPO_ROOT
mkdir -p base_ws/src
vcs import base_ws/src < $REPO_ROOT/ros2_min.repos
vcs import base_ws/src < $REPO_ROOT/rviz2.repos

# Ignore rcl_logging_spdlog package
touch base_ws/src/ros2/rcl_logging/rcl_logging_spdlog/AMENT_IGNORE
touch base_ws/src/ros2/orocos_kdl_vendor/python_orocos_kdl_vendor/AMENT_IGNORE

cd base_ws
colcon build --install-base $REPO_ROOT/ros2 --merge-install --cmake-force-configure --cmake-args -DBUILD_TESTING=NO -DTHIRDPARTY=FORCE -DCOMPILE_TOOLS=NO -DFORCE_BUILD_VENDOR_PKG=ON -DBUILD_MEMORY_TOOLS=OFF -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop -DCMAKE_PREFIX_PATH=$REPO_ROOT/deps
