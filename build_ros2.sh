# Intended usage: At this repo root, do
#
#                        source build_ros2.sh [Platform]
#
# where [Platform] should be either [iOS], [iOS_Simulator], [iOS_Simulator_M1],
# [macCatalyst], [macCatalyst_M1] for iOS or [macOS] for Mac desktop.
# This sets up the basic variables and exports several functions for building.
# Then call `buildRos2Base`, `buildRviz2`, `buildMoveIt2` appropriately.
# The script assumes you have activated the Python environment and already
# builds all dependencies.
#
# Since you must `source` at this repo root, clone this repo where you want to
# install ROS2 and rename the folder (for example `~/usr`) before running.

# The location of this script (the repo root) where supporting files can be found
scriptDir=`pwd`

targetPlatform=$1

colconVerbose=0
colconArgs=(--merge-install --cmake-force-configure)

if [ "$colconVerbose" == "1" ]; then
    colconArgs+=(--executor sequential --event-handlers console_direct+)
fi

prepareVirtualEnv() {
    local ros2PythonEnvPath=$scriptDir/ros2PythonEnv
    python3 -m venv $ros2PythonEnvPath
    source $ros2PythonEnvPath/bin/activate
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
    ros2SystemDependenciesPath=$scriptDir/ros2_$targetPlatform/deps
    echo "Build ros2 base ( assuming dependencies are available at $ros2SystemDependenciesPath )"

    # printPython

    cd $scriptDir
    mkdir -p ros2_ws/src
    cd ros2_ws

    # wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
    vcs import src < $scriptDir/ros2_min.repos

    # Ignore rcl_logging_spdlog package since we are going to force using rcl_logging_noop
    # (i.e. no op => no logging performed) for the logging backend.
    touch src/ros2/rcl_logging/rcl_logging_spdlog/AMENT_IGNORE

    if [ $targetPlatform == "macOS" ]; then
        # For macOS, we install in `base` subdir of `ros2`, to accompany `rviz2` and `moveit2`
        ros2InstallPath=$scriptDir/ros2_$targetPlatform/base
    else
        # For iOS, only `base` is available so we simply use `ros2` as installation dir
        ros2InstallPath=$scriptDir/ros2_$targetPlatform
    fi

    colconArgs+=(--cmake-args -DBUILD_TESTING=NO \
                              -DFORCE_BUILD_VENDOR_PKG=OFF \
                              -DTHIRDPARTY=FORCE \
                              -DCOMPILE_TOOLS=NO \
                              -DBUILD_MEMORY_TOOLS=OFF \
                              -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop)

    if [ $targetPlatform == "macOS" ] || [ $targetPlatform == "macOS_M1" ]; then

        if [ $targetPlatform == "macOS" ]; then
            # For macOS desktop (Intel), we add the CLI tools (ros2 launch) and rclpy as well
            # We probably won't be able to build these for M1 macOS on GitHub Action
            # as there is no way to link with the ARM version of libpython*.dylib there
            # and rclpy and IDL code generation for Python needs that.
            vcs import src < $scriptDir/ros2_cli.repos
        fi

        colconArgs+=(-DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath)

        if [ $targetPlatform == "macOS_M1" ]; then
            colconArgs+=(-DCMAKE_TOOLCHAIN_FILE=$scriptDir/cmake/$targetPlatform.cmake)
        fi

    else
        # For iOS platform, set appropriate toolchain file
        colconArgs+=(-DCMAKE_TOOLCHAIN_FILE=$scriptDir/cmake/$targetPlatform.cmake)

        # For iOS, we need to change the line `#include <net/if_arp.h>` in
        # `src/eProsima/Fast-DDS/src/cpp/utils/IPFinder.cpp` (in the original repo)
        # into `#include <net/ethernet.h>` according to
        # https://stackoverflow.com/questions/10395041/getting-arp-table-on-iphone-ipad
        # as the former header is only available in the macOS SDK.
        #
        # Here, I am commenting out as the repos file used my own fork of `eProsima/Fast-DDS`
        # which already patches the file.
        #
        # sed -i.bak 's/if_arp.h/ethernet.h/g' src/eProsima/Fast-DDS/src/cpp/utils/IPFinder.cpp
    fi

    # --packages-up-to pybind11_vendor
    VERBOSE=$colconVerbose colcon build --install-base $ros2InstallPath "${colconArgs[@]}"
}

patchRviz() {
    # Pass the path to dependencies to rviz_ogre_vendor and orocos_kdl_vendor
    #sed -i.bak "s,CMAKE_ARGS,CMAKE_ARGS\n      -DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath,g" \
    #    src/ros2/rviz/rviz_ogre_vendor/CMakeLists.txt \
    #    src/ros2/orocos_kdl_vendor/orocos_kdl_vendor/CMakeLists.txt

    # Disable most 3rd party dependencies when building libcurl_vendor
    sed -i.bak "s,--with-ssl,--without-ssl --without-librtmp --without-nghttp2 --without-ngtcp2 --without-nghttp3 --without-quiche --without-zlib --without-zstd --without-brotli --disable-ldap,g" src/ros/resource_retriever/libcurl_vendor/CMakeLists.txt

    touch src/ros2/orocos_kdl_vendor/python_orocos_kdl_vendor/AMENT_IGNORE \
          src/ros2/rviz/rviz_visual_testing_framework/AMENT_IGNORE
}

patchRvizM1() {
    # rviz_ogre_vendor forces CMAKE_OSX_ARCHITECTURES=x86_64
    sed -i.bak "s,x86_64,arm64,g" src/ros2/rviz/rviz_ogre_vendor/CMakeLists.txt

    # sed -i.bak "s,CMAKE_ARGS,CMAKE_ARGS\n      -DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath,g" \
        src/ros2/tinyxml_vendor/CMakeLists.txt
}

buildRviz2() {
    rviz2InstallPath=$scriptDir/ros2_$targetPlatform/rviz2
    rviz2SystemDependenciesPath=$scriptDir/ros2_$targetPlatform/deps

    # Source the prebuilt ROS2 base
    # IMPORTANT: GitHub Action uses bash shell!
    source $scriptDir/ros2_$targetPlatform/base/setup.sh

    cd $scriptDir
    mkdir -p rviz2_ws/src
    cd rviz2_ws
    vcs import src < $scriptDir/rviz2.repos

    patchRviz

    VERBOSE=$colconVerbose colcon build --install-base $rviz2InstallPath "${colconArgs[@]}" \
        --cmake-args -DCMAKE_PREFIX_PATH=$rviz2SystemDependenciesPath -DBUILD_TESTING=NO
}

buildMoveIt2() {
    moveit2InstallPath=$scriptDir/ros2_$targetPlatform/moveit2
    depsInstallPath=$scriptDir/ros2_$targetPlatform/deps

    # Ready the workspace
    # I used this
    #     find . -mindepth 1 -maxdepth 1 -exec sh -c "cd {}; git remote --verbose; git branch" \;
    # to find construct the repo file moveit2.repos.
    cd $scriptDir
    mkdir -p moveit2_ws/src
    cd moveit2_ws
    vcs import src < $scriptDir/moveit2.repos

    # macOS does not have sched_setscheduler and we need to fix the const-ness
    # also need to account for https://stackoverflow.com/questions/65397041/apple-clang-why-can-i-not-create-a-time-point-from-stdchrononanoseconds
    cd $scriptDir/moveit2_ws/src/ros2_control
    git apply $scriptDir/ros2_control.patch

    # Fix std::vector<double[6]> and std::random_shuffle removed in C++17
    cd $scriptDir/moveit2_ws/src/moveit2
    git apply $scriptDir/moveit2.patch

    # Disable pilz_industrial_motion_planner for now as it leads to
    #
    # Undefined symbols for architecture x86_64:
    #    "pilz_industrial_motion_planner::LimitsContainer::LimitsContainer()", referenced from:
    #        pilz_industrial_motion_planner::PlanningContextLoader::PlanningContextLoader() in planning_context_loader.cpp.o
    touch moveit_planners/pilz_industrial_motion_planner/AMENT_IGNORE

    # Prepare rviz2 (and base)
    source $scriptDir/ros2_$targetPlatform/rviz2/setup.sh

    # And build
    cd $scriptDir/moveit2_ws
    VERBOSE=$colconVerbose colcon build --install-base $moveit2InstallPath "${colconArgs[@]}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=NO -DCMAKE_PREFIX_PATH="$depsInstallPath"
}

buildTutorials() {
    tutorialsInstallPath=$scriptDir/ros2_$targetPlatform/tutorials
    depsInstallPath=$scriptDir/ros2_$targetPlatform/deps

    cd $scriptDir
    mkdir -p tutorials_ws/src
    cd tutorials_ws
    vcs import src < $scriptDir/tutorials.repos

    sed -i.bak 's/computeGeneric.*}/}/g' src/moveit_task_constructor/core/include/moveit/task_constructor/stage.h
    sed -i.bak 's/rviz::StringProperty/rviz_common::properties::StringProperty/g' src/moveit_task_constructor/visualization/motion_planning_tasks/properties/property_factory.cpp

    # Prepare moveit2
    source $scriptDir/ros2_$targetPlatform/moveit2/setup.sh

    # And build
    VERBOSE=$colconVerbose colcon build --install-base $tutorialsInstallPath "${colconArgs[@]}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=NO -DCMAKE_PREFIX_PATH="$depsInstallPath"
}

buildCartographer() {
    # TODO Implement this
    echo "Build cartographer is not yet implemented"

    # https://github.com/cartographer-project/cartographer
    # https://github.com/cartographer-project/cartographer_ros
    # Need basic ROS2 packages `ros2/rosbag2` and `ros-tooling/keyboard_handler`
    # Need to get extra packages [pcl_msgs](https://github.com/ros-perception/pcl_msgs) and [perception_pcl](https://github.com/ros-perception/perception_pcl) (remember to checkout `ros2` branch)
    # colcon build --merge-install --cmake-args -DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath -DLUA_INCLUDE_DIR=$ros2SystemDependenciesPath/include -DLUA_LIBRARY=$ros2SystemDependenciesPath/lib/liblua.a -DCMAKE_CXX_FLAGS="-L$ros2SystemDependenciesPath/lib -lpcl_common"
}

buildWorkspace() {
    workspaceToBuild=$1
    case $workspaceToBuild in
        "base")
            buildRos2Base;;

        "rviz2")
            buildRviz2;;

        "moveit2")
            buildMoveIt2;;

        "tutorials")
            buildTutorials;;

        *)
            echo "Unknown workspace to build";;
    esac
}
