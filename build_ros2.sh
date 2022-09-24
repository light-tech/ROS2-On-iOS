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
    echo "python3        : found " $(which python3)
    echo "python3-config : found " $(which python3-config)
    echo "    --prefix   : " $(python3-config --prefix)
    echo "    --ldflags  : " $(python3-config --ldflags)
    echo "    --libs     : " $(python3-config --libs)
    echo "    --includes : " $(python3-config --includes)
    echo "    --cflags   : " $(python3-config --cflags)
    # echo "Files in /Users/runner/hostedtoolcache/Python/3.10.7/x64/"
    # find /Users/runner/hostedtoolcache/Python/3.10.7/x64/
}

buildRos2Base() {
    echo "Build ros2 base"

    # Download local ASIO
    # git clone https://github.com/chriskohlhoff/asio

    # Get prebuilt dependencies
    if [ $PLATFORM_TO_BUILD == "macOS" ]; then
        curl -L -o ros2_deps_macOS.tar.xz https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-1.0.1/ros2_deps_macOS.tar.xz
        tar xzf ros2_deps_macOS.tar.xz
    fi

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

        EXTRA_CMAKE_ARGS=(-DCMAKE_PREFIX_PATH=$REPO_ROOT/ros2_deps_macOS)

    else
        # For iOS platform
        EXTRA_CMAKE_ARGS=(-DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/$PLATFORM_TO_BUILD.cmake)

        # Replace if_arp.h header with ethernet.h
        sed -i.bak 's/if_arp.h/ethernet.h/g' src/eProsima/Fast-DDS/src/cpp/utils/IPFinder.cpp
    fi

    # VERBOSE=1 --executor sequential --event-handlers console_direct+
    colcon build --install-base $REPO_ROOT/ros2_$PLATFORM_TO_BUILD \
        --merge-install --cmake-force-configure \
        --cmake-args -DBUILD_TESTING=NO \
            -DTHIRDPARTY=FORCE \
            -DCOMPILE_TOOLS=NO \
            -DFORCE_BUILD_VENDOR_PKG=ON \
            -DBUILD_MEMORY_TOOLS=OFF \
            -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop "${EXTRA_CMAKE_ARGS[@]}"
}

setupROS2base() {
    echo "Prepare ROS2 base and dependencies"

    # Extract previously built dependencies and ROS2 base to save time while we try to build rviz2
    curl -L -o ros2_deps_macOS.tar.xz https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-1.0.1/ros2_deps_macOS.tar.xz \
         -o ros2_macOS.tar.xz https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-1.0.1/ros2_macOS.tar.xz
    tar xzf ros2_deps_macOS.tar.xz
    tar xzf ros2_macOS.tar.xz

    # Source the prebuilt ROS2 base
    # IMPORTANT: GitHub Action uses bash shell!
    source $REPO_ROOT/ros2_macOS/setup.sh
}

buildRviz2() {
    echo "Build rviz2"

    cd $REPO_ROOT
    mkdir -p ros2_ws/src
    cd ros2_ws
    vcs import src < $REPO_ROOT/rviz2.repos

    sed -i.bak "s,CMAKE_ARGS,CMAKE_ARGS\n      -DCMAKE_PREFIX_PATH=$REPO_ROOT/ros2_deps_macOS,g" src/ros2/rviz/rviz_ogre_vendor/CMakeLists.txt
    sed -i.bak 's,CMAKE_ARGS,CMAKE_ARGS\n      -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -I ${CMAKE_PREFIX_PATH}/include/eigen3",g' src/ros2/orocos_kdl_vendor/orocos_kdl_vendor/CMakeLists.txt
    touch src/ros2/orocos_kdl_vendor/python_orocos_kdl_vendor/AMENT_IGNORE

    VERBOSE=1 colcon build --install-base $REPO_ROOT/rviz2_$PLATFORM_TO_BUILD \
        --merge-install --cmake-force-configure \
        --event-handlers console_direct+ \
        --packages-up-to rviz_ogre_vendor \
        --cmake-args -DBUILD_TESTING=NO \
            -DTHIRDPARTY=FORCE \
            -DCOMPILE_TOOLS=NO \
            -DFORCE_BUILD_VENDOR_PKG=ON \
            -DBUILD_MEMORY_TOOLS=OFF \
            -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop -DCMAKE_PREFIX_PATH=$REPO_ROOT/ros2_deps_macOS
}

test -d my_ros2_python_env || prepareVirtualEnv
source my_ros2_python_env/bin/activate
printPython
# buildRos2Base
setupROS2base
buildRviz2
