# Usage:
#
#      build_ros2.sh [Platform]
#
# where [Platform] should be either [iOS], [iOS_Simulator], [iOS_Simulator_M1], [macCatalyst], [macCatalyst_M1]
# or [macOS] for Mac desktop.

REPO_ROOT=`pwd`
PLATFORM_TO_BUILD=$1

buildRos2Base() {
    echo "Build ros2 base"

    # Download local ASIO
    # git clone https://github.com/chriskohlhoff/asio

    cd $REPO_ROOT
    mkdir -p ros2_ws/src
    cd ros2_ws

    # wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
    vcs import src < $REPO_ROOT/ros2_min.repos

    # Ignore rcl_logging_spdlog package
    touch src/ros2/rcl_logging/rcl_logging_spdlog/AMENT_IGNORE

    if [ $PLATFORM_TO_BUILD == "macOS" ]; then
        # For macOS desktop, we add the CLI tools (ros2 launch) and rclpy as well
        vcs import src < $REPO_ROOT/ros2_cli.repos
        EXTRA_CMAKE_ARGS=()

    else
        # For iOS platform
        EXTRA_CMAKE_ARGS=(-DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/$PLATFORM_TO_BUILD.cmake)

        # Replace if_arp.h header with ethernet.h
        sed -i.bak 's/if_arp.h/ethernet.h/g' src/eProsima/Fast-DDS/src/cpp/utils/IPFinder.cpp
    fi

    colcon build --install-base $REPO_ROOT/ros2_$PLATFORM_TO_BUILD \
        --merge-install --cmake-force-configure \
        --cmake-args -DBUILD_TESTING=NO \
            -DTHIRDPARTY=FORCE \
            -DCOMPILE_TOOLS=NO \
            -DFORCE_BUILD_VENDOR_PKG=ON \
            -DBUILD_MEMORY_TOOLS=OFF \
            -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop "${EXTRA_CMAKE_ARGS[@]}"
}

buildRviz2() {
    vcs import ros2_ws/src < $REPO_ROOT/rviz2.repos
    touch ros2_ws/src/ros2/orocos_kdl_vendor/python_orocos_kdl_vendor/AMENT_IGNORE
    echo "Build rviz2"
}

buildRos2Base
