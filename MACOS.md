## Build ROS2 graphical tools for macOS

While there is an option, namely [RoboStack](https://robostack.github.io/GettingStarted.html), to install ROS2 for macOS without having to build it from source, their prebuilt graphical tools such as `rviz2` and `gazebo` do not work.

So let us build ROS2 from source to use on macOS (Intel only) as well.

First we install the required libs to the local location `$REPO_ROOT/deps`.

 1. Get the source code of [FreeType 2](https://download.savannah.gnu.org/releases/freetype/freetype-2.12.1.tar.xz), [Eigen 3](https://eigen.tuxfamily.org/index.php?title=Main_Page), [TinyXML2](https://github.com/leethomason/tinyxml2) and [Bullet](https://github.com/bulletphysics/bullet3). Build and install with

    ```shell
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=$REPO_ROOT/deps/ -DCMAKE_PREFIX_PATH=$REPO_ROOT/deps ..
    cmake --build . --target install
    ```

    For `FreeType2`, add `-DFT_DISABLE_HARFBUZZ=ON` and `-DFT_DISABLE_BZIP2=ON`.

 2. Get [Qt5](https://download.qt.io/archive/qt/5.15/5.15.5/submodules/) (we only need `qtbase`), extract, [patch the source code](https://forum.qt.io/topic/134495/can-t-build-qt-on-monterey-qiosurfacegraphicsbuffer-h-54-32-error-unknown-type-name-cgcolorspaceref-did-you-mean-qcolorspace) and build with

    ```shell
    ./configure -prefix $REPO_ROOT/deps -no-framework
    make
    make install
    ```

 4. Once again, `rviz_ogre_vendor` won't pass all CMake variables to `OGRE`. So change its `CMakeLists.txt` to pass along `-DCMAKE_PREFIX_PATH=$REPO_ROOT/deps/` as well where it can find the dependencies such as `freetype`.

    Likewise for `orocos_kdl_vendor` which relies on `Eigen3Config.cmake` located in `$REPO_ROOT/deps/`.

    Also, disable `python_orocos_kdl_vendor`.

 5. Build ROS2 for macOS using the similar command but set `CMAKE_PREFIX_PATH` to the location where we install the above dependencies

    ```shell
    colcon build --merge-install --cmake-force-configure --cmake-args -DBUILD_TESTING=NO -DTHIRDPARTY=FORCE -DCOMPILE_TOOLS=NO -DFORCE_BUILD_VENDOR_PKG=ON -DBUILD_MEMORY_TOOLS=OFF -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_noop -DCMAKE_PREFIX_PATH=$REPO_ROOT/deps
    ```

**Note**: To build overlaid workspace, you will want to
```shell
colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=$REPO_ROOT/deps
```
to expose our system dependencies.

Now because of [this issue](https://github.com/colcon/colcon-zsh/issues/12) where the generated `setup.zsh` might NOT set the library search path for system libraries (Qt, FreeType2, Bullet ...), tools that depend on our locally built `dylib` in `$REPO_ROOT/deps` will fail to load. (The original instruction works because `homebrew` will put the libraries in globally available location `/usr/local`.)
To actually use your workspace you will need to first
```shell
source YOUR_WORKSPACE/install/setup.zsh
```
and then
```shell
export DYLD_LIBRARY_PATH=$REPO_ROOT/deps/lib:$DYLD_LIBRARY_PATH
```
You must do this in this order because `setup.zsh` does not extend but *overwrite* `DYLD_LIBRARY_PATH`.

## Build cartographer and cartographer-ros packages

Install the dependencies:

 *  [Boost](https://www.boost.org/)

    ```shell
    ./bootstrap.sh --prefix=$REPO_ROOT/deps --without-libraries=python
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
    make install INSTALL_TOP=$REPO_ROOT/deps
    ```

 *  [gmp](https://gmplib.org/)

 *  [mpfr](https://www.mpfr.org/mpfr-current/)

    ```shell
    ./configure --prefix=$REPO_ROOT/deps --with-gmp=$REPO_ROOT/deps
    ```

 *  [SuiteParse](https://github.com/DrTimothyAldenDavis/SuiteSparse): By default, SuiteParse uses 64-bit integer for its `idx_t` (`long long`). This unfortunately clashes with Eigen3's usage of `int`. So change this in the included `SuiteParse/metis-5.1.0/include/metis.h`.

    ```shell
    make install INSTALL=$REPO_ROOT/deps CF="-I $REPO_ROOT/deps/include" LDFLAGS="-L$REPO_ROOT/deps/lib"
    ```

 *  [CERES](http://ceres-solver.org/): Must checkout `2.0.0` to avoid [this issue](https://github.com/cartographer-project/cartographer/issues/1879).

 *  [cairo + pixman](https://www.cairographics.org/download/) with [instruction](https://www.cairographics.org/end_to_end_build_for_mac_os_x/), need [pkgconfig](https://www.freedesktop.org/wiki/Software/pkg-config/) and [libpng](http://www.libpng.org/pub/png/libpng.html) which in turn depend on [zlib](http://zlib.net/)

    ```shell
    export PKG_CONFIG=$REPO_ROOT/deps/bin/pkg-config
    export PKG_CONFIG_PATH=$REPO_ROOT/deps/lib/pkgconfig
    ./configure --prefix=$REPO_ROOT/deps --disable-xlib --disable-ft --enable-svg=no --with-sysroot=$REPO_ROOT/deps
    ```

 *  [flann](https://github.com/flann-lib/flann): pcl's dependency, configure CMake with `-DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF`.

 *  [pcl](https://pointclouds.org/downloads/)

To build [`cartographer`](https://github.com/cartographer-project/cartographer) and [`cartographer-ros`](https://github.com/cartographer-project/cartographer_ros):

```shell
colcon build --merge-install --cmake-args -DCMAKE_PREFIX_PATH=$REPO_ROOT/deps -DLUA_INCLUDE_DIR=$REPO_ROOT/deps/include -DLUA_LIBRARY=$REPO_ROOT/deps/lib/liblua.a -DCMAKE_CXX_FLAGS="-L$REPO_ROOT/deps/lib -lpcl_common"
```

 *  Need basic ROS2 packages `ros2/rosbag2` and `ros-tooling/keyboard_handler`

 *  Need to get extra packages [pcl_msgs](https://github.com/ros-perception/pcl_msgs) and [perception_pcl](https://github.com/ros-perception/perception_pcl) (remember to checkout `ros2` branch)
