# Usage:
#
#      build_ros2.sh [Platform]
#
# where [Platform] should be either [iOS], [iOS_Simulator], [iOS_Simulator_M1], [macCatalyst], [macCatalyst_M1]
# or [macOS] for Mac desktop.

REPO_ROOT=`pwd`
PLATFORM_TO_BUILD=$1

prepareVirtualEnv() {
    python3 -m venv my_ros2_python_env
    source my_ros2_python_env/bin/activate
    python3 -m pip install -r requirements.txt
}

printPython() {
    echo "python3        : " $(which python3)
    echo "python3-config : " $(which python3-config)
    echo "  --prefix     : " $(python3-config --prefix)
    echo "  --ldflags    : " $(python3-config --ldflags)
    echo "  --libs       : " $(python3-config --libs)
    echo "  --includes   : " $(python3-config --includes)
    echo "  --cflags     : " $(python3-config --cflags)
    # echo "Files in /Users/runner/hostedtoolcache/Python/3.10.7/x64/"
    # find /Users/runner/hostedtoolcache/Python/3.10.7/x64/
}


buildRos2Base() {
    echo "Build ros2 base"

    # Download local ASIO
    # git clone https://github.com/chriskohlhoff/asio

    # Get prebuilt dependencies
    # if [ $PLATFORM_TO_BUILD == "macOS" ]; then
    #    curl -L -o ros2_deps_macOS.tar.xz https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-1.0.1/ros2_deps_macOS.tar.xz
    #    tar xzf ros2_deps_macOS.tar.xz
    # fi

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
        # vcs import src < $REPO_ROOT/rviz2.repos
        # touch src/ros2/orocos_kdl_vendor/python_orocos_kdl_vendor/AMENT_IGNORE
        EXTRA_CMAKE_ARGS=() # -DCMAKE_PREFIX_PATH=$REPO_ROOT/ros2_deps_macOS)

    else
        # For iOS platform
        EXTRA_CMAKE_ARGS=(-DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/$PLATFORM_TO_BUILD.cmake)

        # Replace if_arp.h header with ethernet.h
        sed -i.bak 's/if_arp.h/ethernet.h/g' src/eProsima/Fast-DDS/src/cpp/utils/IPFinder.cpp
    fi

    VERBOSE=1 colcon build --install-base $REPO_ROOT/ros2_$PLATFORM_TO_BUILD \
        --merge-install --cmake-force-configure \
        --executor sequential --event-handlers console_direct+ \
        --cmake-args -DBUILD_TESTING=NO \
            -DTHIRDPARTY=FORCE \
            -DCOMPILE_TOOLS=NO \
            -DFORCE_BUILD_VENDOR_PKG=ON \
            -DBUILD_MEMORY_TOOLS=OFF \
            -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop "${EXTRA_CMAKE_ARGS[@]}"
}

prepareVirtualEnv
source my_ros2_python_env/bin/activate
printPython
buildRos2Base
