REPO_ROOT=`pwd`
PLATFORM_TO_BUILD=$1 # Either iOS or iOS_Simulator

# Download local ASIO
# git clone https://github.com/chriskohlhoff/asio

cd $REPO_ROOT
mkdir src
# wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < $REPO_ROOT/ros2_min.repos

# Replace if_arp.h header with ethernet.h
sed -i.bak 's/if_arp.h/ethernet.h/g' $REPO_ROOT/src/eProsima/Fast-DDS/src/cpp/utils/IPFinder.cpp

# Ignore rcl_logging_spdlog package
touch src/ros2/rcl_logging/rcl_logging_spdlog/AMENT_IGNORE

mkdir -p ros2_ws
cd ros2_ws
ln -s ../src src
colcon build --merge-install --cmake-force-configure --cmake-args -DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/$PLATFORM_TO_BUILD.cmake -DBUILD_TESTING=NO -DTHIRDPARTY=FORCE -DCOMPILE_TOOLS=NO -DFORCE_BUILD_VENDOR_PKG=ON -DBUILD_MEMORY_TOOLS=OFF -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop

cd $REPO_ROOT
tar czf ros2_install_$PLATFORM_TO_BUILD.tar.xz ros2_ws/install/
