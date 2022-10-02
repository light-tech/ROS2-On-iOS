## Build ROS2 graphical tools for macOS

While there is an option, namely [RoboStack](https://robostack.github.io/GettingStarted.html), to install ROS2 for macOS without having to build it from source, their prebuilt graphical tools such as `rviz2` and `gazebo` do not work.

So let us build ROS2 from source to use on macOS (Intel only) as well.


## For the impatient

If you want to save yourself one full working day, you can use [our release](https://github.com/light-tech/ROS2-On-iOS/releases) built on GitHub Action server.

 *  If you download the file from a browser, it will be put in quarantine so you need to `xattr -d com.apple.quarantine DOWNLOADED_FILE` before extraction.
 *  To avoid that thank to [this discussion thread](https://developer.apple.com/forums/thread/703523), you could open the terminal and do
    ```shell
    curl -OL https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-macos/ros2_macOS.tar.xz
    ```

After extracting the archive with `tar xzf` and move it to where you want (I usually rename the folder to `ros2` and move it inside `~/usr` in my home folder along with the other Linux-based software), you first need to
```shell
# Tip: Add these commands to your `.zshrc` to have them ready.

export DYLD_LIBRARY_PATH=PATH_TO_EXTRACTED_ROS2/deps/lib:$DYLD_LIBRARY_PATH

source PATH_TO_PYTHON_ENV/bin/activate
source PATH_TO_EXTRACTED_ROS2/setup.zsh
```
and then you can use ROS2 [as usual](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html).

### Note about building and using overlaid workspace

 1. You might want to add `-DCMAKE_PREFIX_PATH` such as
    ```shell
    colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath
    ```
    to expose our system dependencies.
    Here, `$ros2SystemDependenciesPath` should be `PATH_TO_EXTRACTED_ROS2/deps`.

 2. Because of [this issue](https://github.com/colcon/colcon-zsh/issues/12) where the generated `setup.zsh` might NOT set the library search path for system libraries (Qt, FreeType2, Bullet ...), tools that depend on our locally built `dylib` in `$ros2SystemDependenciesPath` will fail to load.
    (The official instruction works because `homebrew` will put the libraries in globally available location `/usr/local`.)
    So to actually use your workspace you might need to `source` it
    ```shell
    source YOUR_WORKSPACE/install/setup.zsh
    ```
    and then
    ```shell
    export DYLD_LIBRARY_PATH=$ros2SystemDependenciesPath/lib:$DYLD_LIBRARY_PATH
    ```
    You must do this in this order because `setup.zsh` might not extend but *overwrite* `DYLD_LIBRARY_PATH`.


## Build rviz2

First we build the required libs to the local location, say `$ros2SystemDependenciesPath`, such as `$REPO_ROOT/deps`, `~/ros2deps` or the commonly used `~/usr`.

 1. Get the source code of [FreeType 2](https://download.savannah.gnu.org/releases/freetype/freetype-2.12.1.tar.xz), [Eigen 3](https://eigen.tuxfamily.org/index.php?title=Main_Page), [TinyXML2](https://github.com/leethomason/tinyxml2) and [Bullet](https://github.com/bulletphysics/bullet3). Build and install with

    ```shell
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=$ros2SystemDependenciesPath -DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath ..
    cmake --build . --target install
    ```

    For `FreeType2`, disable all dependencies with `-DFT_DISABLE_HARFBUZZ=ON -DFT_DISABLE_BZIP2=ON -DFT_DISABLE_PNG=ON -DFT_DISABLE_BROTLI=ON`.

 2. Get [Qt5](https://download.qt.io/archive/qt/5.15/5.15.5/submodules/) (we only need `qtbase`), extract, [patch the source code](https://forum.qt.io/topic/134495/can-t-build-qt-on-monterey-qiosurfacegraphicsbuffer-h-54-32-error-unknown-type-name-cgcolorspaceref-did-you-mean-qcolorspace) and build with

    ```shell
    ./configure -prefix $ros2SystemDependenciesPath -no-framework
    make
    make install
    ```

 4. Once again, `rviz_ogre_vendor` won't pass all CMake variables to `OGRE`. So change its `CMakeLists.txt` to pass along `-DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath` as well where it can find the dependencies such as `freetype`.

    Likewise for `orocos_kdl_vendor` which relies on `Eigen3Config.cmake` located in `$ros2SystemDependenciesPath`.

    The package `libcurl_vendor` by default have a lot of dependencies so either disable most of them or build those you want.

    Also, disable `python_orocos_kdl_vendor` and `rviz_visual_testing_framework`.

 5. Build ROS2 for macOS using the similar command but set `CMAKE_PREFIX_PATH` to the location where we install the above dependencies

    ```shell
    colcon build --merge-install --cmake-force-configure --cmake-args -DBUILD_TESTING=NO -DTHIRDPARTY=FORCE -DCOMPILE_TOOLS=NO -DFORCE_BUILD_VENDOR_PKG=ON -DBUILD_MEMORY_TOOLS=OFF -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop -DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath
    ```


## Build cartographer and cartographer-ros packages

Install the dependencies:

 *  [Boost](https://www.boost.org/)

    ```shell
    ./bootstrap.sh --prefix=$ros2SystemDependenciesPath --without-libraries=python
    ./b2 install
    ```

 *  [googletest](https://github.com/google/googletest)

 *  [protobuf](https://github.com/protocolbuffers/protobuf)

 *  [gflag](https://github.com/gflags/gflags)

 *  [glog](https://github.com/google/glog)

 *  [absl](https://github.com/abseil/abseil-cpp): Add `set (CMAKE_CXX_STANDARD 14)` to the `CMakeLists.txt`

 *  [Lua](https://www.lua.org/download.html)

    ```shell
    make macosx
    make install INSTALL_TOP=$ros2SystemDependenciesPath
    ```

 *  [gmp](https://gmplib.org/)

 *  [mpfr](https://www.mpfr.org/mpfr-current/)

    ```shell
    ./configure --prefix=$ros2SystemDependenciesPath --with-gmp=$ros2SystemDependenciesPath
    ```

 *  [SuiteParse](https://github.com/DrTimothyAldenDavis/SuiteSparse): By default, SuiteParse uses 64-bit integer for its `idx_t` (`long long`). This unfortunately clashes with Eigen3's usage of `int`. So change this in the included `SuiteParse/metis-5.1.0/include/metis.h`.

    ```shell
    make install INSTALL=$ros2SystemDependenciesPath CF="-I $ros2SystemDependenciesPath/include" LDFLAGS="-L$ros2SystemDependenciesPath/lib"
    ```

 *  [CERES](http://ceres-solver.org/): Must checkout `2.0.0` to avoid [this issue](https://github.com/cartographer-project/cartographer/issues/1879).

 *  [cairo + pixman](https://www.cairographics.org/download/) with [instruction](https://www.cairographics.org/end_to_end_build_for_mac_os_x/), need [pkgconfig](https://www.freedesktop.org/wiki/Software/pkg-config/) and [libpng](http://www.libpng.org/pub/png/libpng.html) which in turn depend on [zlib](http://zlib.net/)

    ```shell
    export PKG_CONFIG=$ros2SystemDependenciesPath/bin/pkg-config
    export PKG_CONFIG_PATH=$ros2SystemDependenciesPath/lib/pkgconfig
    ./configure --prefix=$ros2SystemDependenciesPath --disable-xlib --disable-ft --enable-svg=no --with-sysroot=$ros2SystemDependenciesPath
    ```

 *  [flann](https://github.com/flann-lib/flann): pcl's dependency, configure CMake with `-DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF`.

 *  [pcl](https://pointclouds.org/downloads/)

To build [`cartographer`](https://github.com/cartographer-project/cartographer) and [`cartographer-ros`](https://github.com/cartographer-project/cartographer_ros):

```shell
colcon build --merge-install --cmake-args -DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath -DLUA_INCLUDE_DIR=$ros2SystemDependenciesPath/include -DLUA_LIBRARY=$ros2SystemDependenciesPath/lib/liblua.a -DCMAKE_CXX_FLAGS="-L$ros2SystemDependenciesPath/lib -lpcl_common"
```

 *  Need basic ROS2 packages `ros2/rosbag2` and `ros-tooling/keyboard_handler`

 *  Need to get extra packages [pcl_msgs](https://github.com/ros-perception/pcl_msgs) and [perception_pcl](https://github.com/ros-perception/perception_pcl) (remember to checkout `ros2` branch)
