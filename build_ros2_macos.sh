# Build ROS2 Base + Rviz for macOS

REPO_ROOT=`pwd`

# Build dependencies

mkdir deps
cd deps

wget https://download.savannah.gnu.org/releases/freetype/freetype-2.12.1.tar.xz
tar xzf freetype-2.12.1.tar.xz
cd freetype-2.12.1
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$REPO_ROOT/deps/ -DFT_DISABLE_HARFBUZZ=ON -DFT_DISABLE_BZIP2=ON ..
cmake --build . --target install

cd $REPO_ROOT
mkdir -p base_ws/src
vcs import base_ws/src < $REPO_ROOT/ros2_min.repos
vcs import base_ws/src < $REPO_ROOT/rviz2.repos

# Ignore rcl_logging_spdlog package
touch base_ws/src/ros2/rcl_logging/rcl_logging_spdlog/AMENT_IGNORE
touch base_ws/src/ros2/orocos_kdl_vendor/python_orocos_kdl_vendor/AMENT_IGNORE

cd base_ws
colcon build --merge-install --cmake-force-configure --cmake-args -DBUILD_TESTING=NO -DTHIRDPARTY=FORCE -DCOMPILE_TOOLS=NO -DFORCE_BUILD_VENDOR_PKG=ON -DBUILD_MEMORY_TOOLS=OFF -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop  -DCMAKE_PREFIX_PATH=$REPO_ROOT/deps

cd $REPO_ROOT
tar czf ros2_base.tar.xz base_ws/install/
