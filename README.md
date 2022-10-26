# ROS2 on iOS and macOS

Build ROS2 stack for iOS software development.

This valuable experience allows us to build and run ROS2 on Mac **including graphical tools such as RVIZ** as well (see below).

**For the impatient**: Instead of building ROS2 from source (see below), you can download [our prebuilt libs](https://github.com/light-tech/ROS2-On-iOS/releases) and extract it.
Then make a symlink `ros2` pointing to the extracted `ros2_$PLATFORM` where we can find the ROS2 `lib` and `include` headers
```shell
ln -s PATH_TO_EXTRACTED_ros2_$PLATFORM ros2
```
You should of course change the symlink target when switching between building for real iPhone, simulator and Mac Catalyst.
Now we can run the demo application which ports [the official example](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html): Run the app on **two** simulator instances, click on *Start publishing* on one and *Start listening* on the other.

![Minimal Publisher/Subscriber Demo](https://user-images.githubusercontent.com/25411167/184833976-2287a315-0dd8-4d0c-82e6-c42bd7a53d66.mov)

_Note_: To run Mac Catalyst app on your macOS machine, you need to sign the app with your development signing certificate.

**Main guides**:

 * [Build ROS2 from source on macOS](https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html)
 * [Cross compiling ROS2](https://docs.ros.org/en/humble/How-To-Guides/Cross-compilation.html)
 * [ROS2 Architecture](https://docs.ros.org/en/humble/Concepts/About-Internal-Interfaces.html)

## Tools preparation

We are going to need

 * CMake **3.23** until [this issue](https://github.com/ament/ament_cmake/pull/395) is sorted out
 * Python **3.10**
 * Xcode + Command Line tools

installed.

If you have other environment management such as Anaconda, remember to **DEACTIVATE** them.
Then create and activate a Python virtual environment for ease of package installation

```shell
python3 -m venv ros2PythonEnv
source ros2PythonEnv/bin/activate
python3 -m pip install -r requirements.txt
```

## Build ROS2 core communication packages for iOS

To build for iOS simulator on Intel Macs, use the script [`build_ros2.sh`](build_ros2.sh).
You might need to change some variable in the script to match your system.
Activate your Python environment then execute
```shell
 # change iOS to iOS_Simulator if you want to test ROS2 on the iPhone simulator
source build_ros2.sh iOS

buildRos2Base
```
at the **root of this repo** (the script must be `source`d there so you should clone this repo where you want to put your ROS2 installation).

The key command in the script is
```shell
colcon build --merge-install \
    --cmake-force-configure \
    --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/iOS.cmake \
        -DBUILD_TESTING=NO \
        -DTHIRDPARTY=FORCE \
        -DCOMPILE_TOOLS=NO \
        -DFORCE_BUILD_VENDOR_PKG=ON \
        -DBUILD_MEMORY_TOOLS=OFF \
        -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop
```

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

## Build ROS2 with graphical tools for macOS

While there is an option, namely [RoboStack](https://robostack.github.io/GettingStarted.html), to install ROS2 for macOS without having to build it from source, their prebuilt graphical tools such as `rviz2` and `gazebo` do not work.

So I find a way to build ROS2 from source to use on macOS (Intel only) as well.
There is **no need to disable SIP** like the official instruction suggested.
I also build pretty much all dependencies so **Homebrew isn't needed either**.

Unlike iOS, you first need to build the dependencies using [`build_deps.sh`](build_deps.sh) and install [XQuartz](https://www.xquartz.org/) if you want to build MoveIt2.
Then similar to iOS,
```shell
source build_ros2.sh macOS

buildRos2Base
buildRviz2       # Run if you need Rviz2 (depends on base)
buildMoveIt2     # Run if you need MoveIt2 (depends on Rviz2)
```

See [`build_ros2.sh`](build_ros2.sh) for more details.

## Using our prebuilt ROS2 for macOS

If you want to save yourself one full working day, you can use [our release](https://github.com/light-tech/ROS2-On-iOS/releases) built using GitHub Action.

**Note**: We did not manage to build native ROS2 for ARM64 Mac but you can still use the Intel version thank to [Rosetta 2](https://support.apple.com/en-us/HT211861).
In that case, you will need to install `x86_64` version of Python.

 *  If you download the file from a browser, it will be put in quarantine so you need to `xattr -d com.apple.quarantine DOWNLOADED_FILE` before extraction.
 *  To avoid that, you could open the terminal and do
    ```shell
    curl -OL https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-1.2/deps_macOS.tar.xz \
         -O https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-1.2/ros2_macOS.tar.xz \
         -O https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-1.2/rviz2_macOS.tar.xz \
         -O https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-1.2/moveit2_macOS.tar.xz
    ```
    instead.

After extracting the archives with `tar xzf` and move the resulting `ros2_macOS` to where you want (I usually rename it to `ros2` and move it inside `~/usr` in my home folder along with the other Linux-based software), you need to **fix hardcoded paths** to match YOUR system as `colcon` (or maybe CMake) unfortunately hardcoded a lot of paths (mostly concerning Python-based stuffs) in generated files such as CMake and package config files.

 *  To do that, run our script `fix_hardcoded_paths.sh` and you will be prompted with the relevant information.
 *  The script is not exhaustive so you will need to manually fix the path such as the path to `python3` in `bin/ros2` so you can run `ros2` command line such as `ros2 launch` and `ros2 topic list`.

Next, add this to your `.zshrc` (appropriate modification required)
```shell
activateROS2() {
    # So that ros2 can find the Python framework that you installed yourself.
    # Change the path /Library/Frameworks appropriately to fit your system.
    export DYLD_FRAMEWORK_PATH=/Library/Frameworks:$DYLD_FRAMEWORK_PATH

    source PATH_TO_PYTHON_ENV/bin/activate
    source PATH_TO_EXTRACTED_ROS2/moveit2/setup.zsh

    # Add the location for ROS2 to find prebuilt dependencies
    # Do this AFTER source setup.zsh because of https://github.com/colcon/colcon-zsh/issues/12
    export DYLD_LIBRARY_PATH=PATH_TO_EXTRACTED_ROS2/deps/lib:$DYLD_LIBRARY_PATH
}
```
so that anytime you need ROS2, you can simply execute `activateROS2` in the terminal.

Now you can start using ROS2 [as usual](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html).

 *  Try following the various tutorials such as [the image transport tutorial](https://github.com/ros-perception/image_transport_tutorials) and the [URDF tutorial](https://github.com/ros/urdf_tutorial).

## ROS Development Tip

Define functions `buildCMake` and `buildColcon` in your `.zshrc` such as
```shell
buildCMake() {
    mkdir _build && cd _build
    cmake -DCMAKE_INSTALL_PREFIX=$HOME/usr/ros2/deps -DCMAKE_PREFIX_PATH="$HOME/usr/ros2/deps" "$@" ..
    cmake --build . --config Release --target install
}

buildColcon() {
    colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=$PATH_TO_EXTRACTED_ROS2/deps -DBUILD_TESTING=NO
}
```
to avoid typing (copying and pasting) long chains of shell commands in the terminal.
