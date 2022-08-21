# ROS2 on iOS

Build ROS2 stack for iOS software development.

**For the impatient**: Instead of building ROS2 from source (see below), you can download [our prebuilt libs](https://github.com/light-tech/ROS2-On-iOS/releases) and extract it to the root of the repo with `tar xzf`.
Then make a symlink `ros2` pointing to `ros2_ws/install` where we can find the `lib` and `include` headers
```shell
ln -s ros2_ws/install ros2
```
You should change the symlink target when switching between building for real iPhone and for simulator.
You will need to delete the symlinks `libfastcdr.1.dylib` and `libfastrtps.2.6.dylib` in `install/lib` and rename the `libfastcdr.1.0.24.dylib` and `libfastrtps.2.6.2.dylib` to those.
Now we can run the demo application: Run the app on **two** simulator instances, click on *Start publishing* on one and *Start listening* on the other.

![Minimal Publisher/Subscriber Demo](https://user-images.githubusercontent.com/25411167/184833976-2287a315-0dd8-4d0c-82e6-c42bd7a53d66.mov)

**Main guides**:

 * [Build ROS2 from source on macOS](https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html)
 * [Cross compiling ROS2](https://docs.ros.org/en/humble/How-To-Guides/Cross-compilation.html)
 * [Architecture](https://docs.ros.org/en/humble/Concepts/About-Internal-Interfaces.html)

## Tools preparation

We are going to need

 * CMake **3.23** until [this](https://github.com/ament/ament_cmake/pull/395) is sorted out
 * Python **3.10**
 * Xcode + Command Line tools

installed. All subsequent shell commands are to be done at `$REPO_ROOT`.

Also, create a Python virtual environment for ease of working

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

If you have other environment management such as Anaconda, remember to deactivate them.

## Source and workspace preparation

We are going to disable most packages and add back only those that are necessary.
To do that,

 1. I deviate from the guide to get the sources into `unused_src` instead

    ```shell
    mkdir unused_src
    vcs import unused_src < ros2.repos
    touch unused_src/AMENT_IGNORE       # So that colcon will not find packages in unused_src
    ```

 2. Move the minimal packages `ros2/rcl`, `ros2/rmw`, `ros2/rosidl` to `src` to start

    ```shell
    mkdir -p src/ros2
    mv unused_src/ament src/
    mv unused_src/eProsima src/
    mv unused_src/ros2/rcl src/ros2/
    mv unused_src/ros2/rmw src/ros2/
    mv unused_src/ros2/rosidl src/ros2/
    mv unused_src/ros2/common_interfaces src/ros2/
    ```

    It is good to have `ros2/common_interfaces` as it provides the commonly used packages such as `std_msgs`.

    Certain packages such as `rviz` are obviously not easy to port.
    We will run `colcon` commands (below) and add back more dependent packages.

    **Tip**: Do `find unused_src -name PKG_NAME` in `unused_src` to find the repository containing the needed package.

 3. The minimal list of repositories is available in the file `ros2_min.repos`.
    So of course, one can simply import this instead.
    Here, I choose to use a single RMW implementation (via Fast-DDS).

 4. Create a new workspace and link the `src` in

    ```shell
    mkdir -p ros2_ws
    cd ros2_ws
    ln -s $REPO_ROOT/src src
    ```

 5. It is good to have multiple workspace. In the base workspace, build `ament` packages and `source` it. Then in the next workspace, add `rcl` and its dependencies and in yet another, add other desired packages such as `rclcpp`. This way, one can minimize the amount of packages to rebuild.

    For `rclcpp`, need to add `class_loader`, `console_bridge_vendor` and `libstatistics_collector`.
    Added these using `opendiff ros2.repos ros2_min.repos`.

## Build ROS2 for iOS simulator

Before building, we need to change the line `#include <net/if_arp.h>` in `src/eProsima/Fast-DDS/src/cpp/utils/IPFinder.cpp` into `#include <net/ethernet.h>` according to [this](https://stackoverflow.com/questions/10395041/getting-arp-table-on-iphone-ipad) as the original header is only available in the macOS SDK.

We also disable `rcl_logging_spdlog` by
```shell
touch src/ros2/rcl_logging/rcl_logging_spdlog/AMENT_IGNORE
```

To build for iOS simulator on Intel Macs, do

```shell
cd ros2_ws

#Prepend with VERBOSE=1 and add --executor sequential --event-handlers console_direct+ while debugging
colcon build --merge-install \
    --cmake-force-configure \
    --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/iOS_Simulator.cmake \
        -DBUILD_TESTING=NO \
        -DTHIRDPARTY=FORCE \
        -DCOMPILE_TOOLS=NO \
        -DFORCE_BUILD_VENDOR_PKG=ON \
        -DBUILD_MEMORY_TOOLS=OFF \
        -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop
```

For M1 Mac, you would need to change `iOS_Simulator.cmake` appropriately.

After my tons of failures, here is what went going on behind the above command:

 1. The classical method to CMake for iOS simulator in my other projects such as [LibGit2](https://github.com/light-tech/LibGit2-On-iOS/) is to set the appropriate `CMAKE_OSX_SYSROOT` and `CMAKE_OSX_ARCHITECTURES`.

    **Note**: The right way to get `clang`  to compile for iOS simulator is to set to `CMAKE_C_FLAGS` and `CMAKE_CXX_FLAGS`, namely `-target x86_64-apple-ios-simulator -mios-version-min=14.0` together with `-isysroot` to find the standard headers.

    However, that is not going to work here.
    The reason is because of the `***_vendor` packages which are proxy packages for 3rd party libraries such as `libyaml`: their CMake files fetch the library and passing to their CMake with the "right" options.
    Checking for example `libyaml_vendor/CMakeLists.txt` reveals that it does NOT pass along all our `CMAKE_***` options except for `CMAKE_C_FLAGS` and even force `BUILD_SHARED_LIBS=ON`.
    We need to pass along all desired options in `-DCMAKE_TOOLCHAIN_FILE` and in that file, we set up all the desired CMake variables.

    **Note**: Even this is not going to be enough for some package such as `mimick_vendor` as it sets its internal variable `_ARCH` **BEFORE** considering the toolchain file!

 2. `BUILD_TESTING=NO` to disable all test packages, without this, we are going to need a ton of third party packages such as `ament/google_benchmark_vendor`, `ament/uncrustify_vendor`, `ament/googletest`, `osrf/osrf_testing_tools_cpp`, `ros2/performance_test_fixture` and `ros2/mimick_vendor` and account for a ton of CMake issues

 3. `THIRDPARTY=FORCE` is to ask eProsima Fast-DDS to build and use its own dependencies (`asio`, ...) so that we do not have to install it to the system with Homebrew

 4. `COMPILE_TOOLS=NO` for `Fast-DDS` to **NOT** compile the fast discovery server executable.

 5. `FORCE_BUILD_VENDOR_PKG=ON` is to build and use the `*_vendor` packages such as `libyaml_vendor`, again so that we do not have to install it to the system with Homebrew

 6. `BUILD_MEMORY_TOOLS=OFF` is for `foonathan_memory` to disable building `nodesize_db` program

 7. `RCL_LOGGING_IMPLEMENTATION=rcl_logging_noop` is to select the logging backend for `rcl_logging`.
    Here, I am using `noop` one to avoid one more dependency `spdlog_vendor`.

 8. ROS2 depends significantly on dynamic linking. Do NOT add `BUILD_SHARED_LIBS=NO`, contrary to my other project [LLVM](https://github.com/light-tech/LLVM-On-iOS/) where building static libs is needed!
