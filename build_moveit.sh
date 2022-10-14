REPO_ROOT=`pwd`

ros2PythonEnvPath=$REPO_ROOT/ros2PythonEnv

depsSrcDownloadPath=$REPO_ROOT/deps_download
depsSrcExtractPath=$REPO_ROOT/deps_extract
depsInstallPath=$REPO_ROOT/ros2_macOS/deps

getMoveItDeps() {
    mkdir -p $depsSrcDownloadPath
    cd $depsSrcDownloadPath
    curl -s -L -o fcl.tar.gz https://github.com/flexible-collision-library/fcl/archive/refs/tags/0.7.0.tar.gz \
        -o ccd.tar.gz https://github.com/danfis/libccd/archive/refs/tags/v2.1.tar.gz \
        -o octomap.tar.gz https://github.com/OctoMap/octomap/archive/refs/tags/v1.9.6.tar.gz \
        -o qhull.tgz http://www.qhull.org/download/qhull-2020-src-8.0.2.tgz \
        -o assimp.tar.gz https://github.com/assimp/assimp/archive/refs/tags/v5.2.5.tar.gz \
        -o ruckig.tar.gz https://github.com/pantor/ruckig/archive/refs/tags/v0.8.4.tar.gz \
        -o glew.tgz https://github.com/nigels-com/glew/releases/download/glew-2.2.0/glew-2.2.0.tgz \
        -o freeglut.tar.gz https://github.com/FreeGLUTProject/freeglut/releases/download/v3.4.0/freeglut-3.4.0.tar.gz \
        -o openssl.tar.gz https://github.com/openssl/openssl/archive/refs/tags/openssl-3.0.6.tar.gz \
        -o omplcore.tar.gz https://github.com/ompl/ompl/archive/1.5.2.tar.gz \
        -o openmp.tar.xz https://github.com/llvm/llvm-project/releases/download/llvmorg-14.0.6/openmp-14.0.6.src.tar.xz
}

extractMoveItDeps() {
    mkdir -p $depsSrcExtractPath
    cd $depsSrcExtractPath
    local src_files=`ls $depsSrcDownloadPath`
    ls -all $depsSrcDownloadPath
    for f in $src_files; do
        tar xzf $depsSrcDownloadPath/$f
    done
}

buildCMake() {
    mkdir _build && cd _build
    cmake -DCMAKE_INSTALL_PREFIX=$depsInstallPath -DCMAKE_PREFIX_PATH="$depsInstallPath" "$@" ..
    cmake --build . --config Release --target install
}

buildMoveItDeps() {
    cd $depsSrcExtractPath/libccd-2.1 && buildCMake
    cd $depsSrcExtractPath/octomap-1.9.6 && buildCMake

    # For octomap, we need to manually add
    #   IMPORTED_LOCATION "${_IMPORT_PREFIX}/lib/libocto(map|math).1.9.6.dylib"
    # appropriately to the two commands
    #   set_target_properties(octomath PROPERTIES ...)
    #   set_target_properties(octomap PROPERTIES ...)
    # in the generated share/octomap/octomap-targets.cmake. Without this, building fcl will fail with
    #
    # CMake Error in CMakeLists.txt:
    #   IMPORTED_LOCATION not set for imported target "octomap" configuration
    #   "Release".

    sed -i.bak -E 's,set_target_properties\(octo(math|map) PROPERTIES,set_target_properties(octo\1 PROPERTIES\n  IMPORTED_LOCATION "\${_IMPORT_PREFIX}/lib/libocto\1.1.9.6.dylib",' $depsInstallPath/share/octomap/octomap-targets.cmake

    # FCL depends on octomap and libccd
    cd $depsSrcExtractPath/fcl-0.7.0 && buildCMake -DBUILD_TESTING=NO
    cd $depsSrcExtractPath/qhull-2020.2 && buildCMake
    cd $depsSrcExtractPath/assimp-5.2.5 && buildCMake
    cd $depsSrcExtractPath/ruckig-0.8.4 && buildCMake
    cd $depsSrcExtractPath/glew-2.2.0/build/cmake && buildCMake

    # FreeGLUT needs X11 (provided by XQuartz for macOS)
    # For some reason the X11 include path is not added so we force add it
    cd $depsSrcExtractPath/freeglut-3.4.0 && buildCMake -DFREEGLUT_BUILD_DEMOS=NO -DCMAKE_C_FLAGS="-isystem /usr/X11R6/include"

    cd $depsSrcExtractPath/openssl-openssl-3.0.6 && ./Configure --prefix=$depsInstallPath && make && make install
    cd $depsSrcExtractPath/ompl-1.5.2 && buildCMake
    cd $depsSrcExtractPath/openmp-14.0.6.src && buildCMake
}

prepareVirtualEnv() {
    python3 -m venv $ros2PythonEnvPath
    source $ros2PythonEnvPath/bin/activate
    python3 -m pip install -r requirements.txt
}

buildMoveIt() {
    echo "Build MoveIt2 packages"

    # Ready the workspace
    # I used this
    #     find . -mindepth 1 -maxdepth 1 -exec sh -c "cd {}; git remote --verbose; git branch" \;
    # to find construct the repo file moveit2.repos.
    cd $REPO_ROOT
    mkdir -p moveit2_ws/src
    cd moveit2_ws
    vcs import src < $REPO_ROOT/moveit2.repos

    # macOS does not have sched_setscheduler and we need to fix the const-ness
    # also need to account for https://stackoverflow.com/questions/65397041/apple-clang-why-can-i-not-create-a-time-point-from-stdchrononanoseconds
    cd $REPO_ROOT/moveit2_ws/src/ros2_control
    git apply $REPO_ROOT/ros2_control.patch

    # Fix std::vector<double[6]> and std::random_shuffle removed in C++17
    cd $REPO_ROOT/moveit2_ws/src/moveit2
    git apply $REPO_ROOT/moveit2.patch

    # Disable pilz_industrial_motion_planner for now as it leads to
    #
    # Undefined symbols for architecture x86_64:
    #    "pilz_industrial_motion_planner::LimitsContainer::LimitsContainer()", referenced from:
    #        pilz_industrial_motion_planner::PlanningContextLoader::PlanningContextLoader() in planning_context_loader.cpp.o
    touch moveit_planners/pilz_industrial_motion_planner/AMENT_IGNORE

    # Prepare base
    source $REPO_ROOT/ros2_macOS/setup.sh

    # And build
    cd $REPO_ROOT/moveit2_ws
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=NO -DCMAKE_PREFIX_PATH="$depsInstallPath"
}

#getMoveItDeps
#extractMoveItDeps
#buildMoveItDeps

test -d $ros2PythonEnvPath && source $ros2PythonEnvPath/bin/activate || prepareVirtualEnv
buildMoveIt
