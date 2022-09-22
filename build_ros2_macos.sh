# Build ROS2 Base + Rviz for macOS
# Assuming dependencies are built prior and installed in ros2_deps_macOS

REPO_ROOT=`pwd`

cd $REPO_ROOT
mkdir -p ros2_ws/src
vcs import ros2_ws/src < $REPO_ROOT/ros2_min.repos

# Ignore rcl_logging_spdlog package
touch ros2_ws/src/ros2/rcl_logging/rcl_logging_spdlog/AMENT_IGNORE

cd ros2_ws
colcon build --install-base $REPO_ROOT/ros2_macOS --merge-install --cmake-force-configure --cmake-args -DBUILD_TESTING=NO -DTHIRDPARTY=FORCE -DCOMPILE_TOOLS=NO -DFORCE_BUILD_VENDOR_PKG=ON -DBUILD_MEMORY_TOOLS=OFF -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop # -DCMAKE_PREFIX_PATH=$REPO_ROOT/ros2_deps_macOS

# vcs import ros2_ws/src < $REPO_ROOT/rviz2.repos
# touch ros2_ws/src/ros2/orocos_kdl_vendor/python_orocos_kdl_vendor/AMENT_IGNORE
