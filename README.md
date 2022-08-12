# ROS2 on iOS

Build ROS2 stack for iOS software development.

## Build ROS2 for macOS

Guides: https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html

We are going to need CMake 3.23 until [this](https://github.com/ament/ament_cmake/pull/395) is sorted out, Python and Xcode installed.

 1. Create a new Python virtual environment
    ```shell
    export MY_PYTHON_ENV=usr_local
    python3 -m venv $MY_PYTHON_ENV
    ```
    Here I use `usr_local` as it is intended to act like `/usr/local/`.

 2. Activate the environment
    ```shell
    source $MY_PYTHON_ENV/bin/activate
    ```

 3. Install the packages
    ```shell
    python3 -m pip install -r requirements.txt
    ```

 4. Compile `googletest`:
    ```shell
    cd usr_local/lib/python3.10/site-packages/mypyc/external/googletest/make
    make
    ```

 5. We are going to disable most packages and add back only those that are necessary.
    To do that, I deviate from the guide to get the sources into `unused_src` instead
    ```shell
    mkdir unused_src
    vcs import unused_src < ros2.repos
    touch unused_src/AMENT_IGNORE       # So that colcon will not find packages in unused_src
    ```
    and move the minimal packages to `src` to start
    ```shell
    mkdir -p src/ros2
    mv unused_src/ament src/
    mv unused_src/eProsima src/
    mv unused_src/ros2/rcl src/ros2/
    ```
    The most important package is `ros2/rcl`. Certain packages such as `rviz` are obviously not easy to port.

 6. Keep doing
    ```shell
    colcon build --symlink-install --packages-skip-by-dep python_qt_binding --cmake-args -DTHIRDPARTY=FORCE -DFORCE_BUILD_VENDOR_PKG=ON -DBUILD_SHARED_LIBS=NO -DBUILD_TESTING=NO
    ```
    and adding back dependent packages until we can successfully build.

    **Tip**: Do `find unused_src -name PKG_NAME` in `unused_src` to find the repository containing the needed package.

     - `FORCE_BUILD_VENDOR_PKG=ON` is to build the `*_vendor` packages such as `tinyxml2_vendor` so that we do not have to install it to the system with Homebrew.
     - `THIRDPARTY=FORCE` is to ask eProsima Fast-DDS to build and use its own dependencies (`asio`, ...)
     - `BUILD_TESTING=NO` to disable all test packages (originally, I have to disable the package `src/ros2/rcl_interfaces/test_msgs` by manually adding `AMENT_IGNORE` but after checking some CMakeLists.txt, I found this great option to disable all test packages.)

 7. The final hurdle to overcome is that when building static libraries (desired for iOS), we run into the error of having both `typesupport` package (namely `rosidl_typesupport_fastrtps` and `rosidl/rosidl_typesupport_introspection`) which is impossible because we need dynamic libraries then to load them at runtime when needed.

    > CMake Error at .../install/rosidl_typesupport_c/share/rosidl_typesupport_c/cmake/rosidl_typesupport_c_generate_interfaces.cmake:114 (message):
    >   Multiple typesupports
    >   [rosidl_typesupport_fastrtps_c;rosidl_typesupport_introspection_c] but
    >   static linking was requested

    According to [this](https://docs.ros.org/en/humble/Concepts/About-Internal-Interfaces.html), the package `ros2/rosidl_typesupport` basically allows for resolving the right dynamic library `rosidl_typesupport_***` at run time.
    We only want **Static Type Support** here. So we remove the universal `rosidl/rosidl_typesupport_introspection_(c|cpp)` package for Dynamic Type Support as well as the implementation `rmw_fastrtps_dynamic_cpp`.
    We need to update `rmw_fastrtps/rmw_fastrtps_shared_cpp/CMakeLists.txt` to take out `rosidl_typesupport_introspection_***` dependencies.
    Also need to comment out the relevant code in the source code `TypeSupport_Impl.cpp` of `rmw_fastrtps_shared_cpp`.
    See https://github.com/ros2/ros2/issues/468 and https://github.com/ros2/ros2/issues/306 and https://github.com/ros2/rosidl_typesupport/pull/10 .

    Other relevant pages are https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html and https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html and https://docs.ros.org/en/humble/Installation/DDS-Implementations.html

According to `find src -maxdepth 2`, my final `src` directory contains:
```
src
src/osrf
src/osrf/osrf_testing_tools_cpp
src/ament
src/ament/uncrustify_vendor
src/ament/ament_cmake
src/ament/ament_package
src/ament/googletest
src/ament/google_benchmark_vendor
src/ament/ament_index
src/ament/ament_lint
src/ros-tracing
src/ros-tracing/ros2_tracing
src/ros2
src/ros2/rcl
src/ros2/ament_cmake_ros
src/ros2/rmw_dds_common
src/ros2/rcutils
src/ros2/launch
src/ros2/performance_test_fixture
src/ros2/rmw
src/ros2/rosidl_typesupport_fastrtps
src/ros2/rosidl_defaults
src/ros2/mimick_vendor
src/ros2/rosidl_typesupport
src/ros2/rmw_fastrtps
src/ros2/rmw_implementation
src/ros2/rosidl
src/ros2/rcl_interfaces
src/ros2/spdlog_vendor
src/ros2/test_interface_files
src/ros2/rcpputils
src/ros2/rcl_logging
src/ros2/python_cmake_module
src/ros2/libyaml_vendor
src/ros2/unique_identifier_msgs
src/eProsima
src/eProsima/foonathan_memory_vendor
src/eProsima/Fast-DDS
src/eProsima/Fast-CDR
```

## Build ROS2 for iOS

Guide: https://docs.ros.org/en/humble/How-To-Guides/Cross-compilation.html

 1. Make a new workspace and link source to save us the clone
    ```shell
    mkdir -p ros2_ios
    cd ros2_ios
    ln -s ../src src
    ```

 2. Grab `CMAKE_TOOLCHAIN_FILE` for iOS from
    ```shell
    curl https://github.com/llvm-mirror/llvm/raw/master/cmake/platforms/iOS.cmake >iOS.cmake
    ```

 3. Build with iOS SDK
    ```shell
    SYSROOT=`xcodebuild -version -sdk iphoneos Path`
    colcon build --merge-install --cmake-force-configure --cmake-args -DTHIRDPARTY=FORCE -DFORCE_BUILD_VENDOR_PKG=ON -DBUILD_SHARED_LIBS=NO -DBUILD_TESTING=NO -DCOMPILE_TOOLS=NO -DCMAKE_OSX_SYSROOT=$SYSROOT -DCMAKE_OSX_ARCHITECTURES=arm64 # -DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/iOS.cmake --executor sequential --event-handlers console_direct+
    ```

    Compared to macOS, we further

     - Add `-DCOMPILE_TOOLS=NO` for `Fast-DDS` to **NOT** compile the fast discovery server executable. Without this, we are running into the _"compiling for iPhone but linking with macOS library"_ issue.
     - Change `#include <net/if_arp.h>` (in `src/eProsima/Fast-DDS/src/cpp/utils/IPFinder.cpp`) into `#include <net/ethernet.h>` according to [this](https://stackoverflow.com/questions/10395041/getting-arp-table-on-iphone-ipad).
     - Ignore the packages `ament/google_benchmark_vendor`, `ament/uncrustify_vendor`, `ament/googletest`, `osrf/osrf_testing_tools_cpp` and `ros2/performance_test_fixture`.

 4. Similarly for iPhone simulator, we do
    ```shell
    SYSROOT=`xcodebuild -version -sdk iphonesimulator Path`
    colcon build --merge-install --cmake-force-configure --cmake-args -DTHIRDPARTY=FORCE -DFORCE_BUILD_VENDOR_PKG=ON -DBUILD_SHARED_LIBS=NO -DBUILD_TESTING=NO -DCOMPILE_TOOLS=NO -DCMAKE_SYSROOT=$SYSROOT -DCMAKE_OSX_SYSROOT=$SYSROOT -DCMAKE_OSX_ARCHITECTURES=x86_64
    ```
