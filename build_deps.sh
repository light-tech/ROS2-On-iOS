# Platform to build [iOS, iOS_Simulator, macOS]
PLATFORM=$1

# This repo root (where this script is located)
REPO_ROOT=`pwd`

# The sysroot of the dependencies we built
DEPS_SYSROOT=$REPO_ROOT/deps_$PLATFORM

# Prefix where we built Qt for host machine
QT_HOST_PREFIX=$REPO_ROOT/host_deps/

# Where we download the source archives
DOWNLOAD_PATH=$REPO_ROOT/deps_download/

# Location to extract the source
SRC_PATH=$REPO_ROOT/deps_src/

export PATH=$PATH:$DEPS_SYSROOT/bin

function getAndExtractSource() {
    mkdir -p $DOWNLOAD_PATH
    cd $DOWNLOAD_PATH
    # Need -L to download github releases according to https://stackoverflow.com/questions/46060010/download-github-release-with-curl
    curl -s -L -o abseil-cpp.tar.gz https://github.com/abseil/abseil-cpp/archive/refs/tags/20220623.0.tar.gz \
         -o boost.tar.gz https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.gz \
         -o gflags.tar.gz https://github.com/gflags/gflags/archive/refs/tags/v2.2.2.tar.gz \
         -o cairo.tar.xz https://www.cairographics.org/releases/cairo-1.16.0.tar.xz \
         -o pixman.tar.gz https://cairographics.org/releases/pixman-0.40.0.tar.gz \
         -o bullet3.tar.gz https://github.com/bulletphysics/bullet3/archive/refs/tags/3.24.tar.gz \
         -o glog.tar.gz https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz \
         -o pkg-config.tar.gz https://pkgconfig.freedesktop.org/releases/pkg-config-0.29.2.tar.gz \
         -o gmp.tar.xz https://gmplib.org/download/gmp/gmp-6.2.1.tar.xz \
         -o protobuf.tar.gz https://github.com/protocolbuffers/protobuf/releases/download/v21.5/protobuf-cpp-3.21.5.tar.gz \
         -o libpng.tar.xz https://download.sourceforge.net/libpng/libpng-1.6.37.tar.xz \
         -o eigen.tar.bz2 https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.bz2 \
         -o lua.tar.gz https://www.lua.org/ftp/lua-5.4.4.tar.gz \
         -o tinyxml2.tar.gz https://github.com/leethomason/tinyxml2/archive/refs/tags/9.0.0.tar.gz \
         -o flann.tar.gz https://github.com/flann-lib/flann/archive/refs/tags/1.9.1.tar.gz \
         -o mpfr.tar.xz https://www.mpfr.org/mpfr-current/mpfr-4.1.0.tar.xz \
         -o zlib.tar.xz https://zlib.net/zlib-1.2.12.tar.xz \
         -o freetype.tar.xz https://download.savannah.gnu.org/releases/freetype/freetype-2.12.1.tar.xz \
         -o pcl.tar.gz https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.12.1/source.tar.gz \
         -o googletest.tar.gz https://github.com/google/googletest/archive/refs/tags/release-1.12.1.tar.gz \
         -o SuiteSparse.tar.gz https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/refs/tags/v5.12.0.tar.gz \
         -o ceres-solver.tar.gz http://ceres-solver.org/ceres-solver-2.0.0.tar.gz \
         -o qtbase.tar.gz https://download.qt.io/archive/qt/5.15/5.15.5/submodules/qtbase-everywhere-opensource-src-5.15.5.tar.xz
         #-o autoconf.tar.gz http://ftpmirror.gnu.org/autoconf/autoconf-2.69.tar.gz \
         #-o automake.tar.gz http://ftpmirror.gnu.org/automake/automake-1.15.tar.gz \
         #-o libtool.tar.gz http://ftpmirror.gnu.org/libtool/libtool-2.4.6.tar.gz \
         #-o mm-common.tar.gz https://github.com/GNOME/mm-common/archive/refs/tags/1.0.4.tar.gz

    mkdir -p $SRC_PATH
    cd $SRC_PATH
    local src_files=`ls $DOWNLOAD_PATH`
    ls -all $DOWNLOAD_PATH
    for f in $src_files; do
        tar xzf $DOWNLOAD_PATH/$f
    done
}

function setupPlatform() {
    case $PLATFORM in
        "iOS")
            ARCH=arm64
            SYSROOT=`xcodebuild -version -sdk iphoneos Path`
            MAKE_HOST_TRIPLE_ZLIB=arm-apple-darwin # For zlib, pixman and cairo we must use arm-apple and not arm64-apple thank to  https://gist.github.com/jvcleave/9d78de9bb27434bde2b0c3a1af355d9c
            MAKE_HOST_TRIPLE=aarch64-apple-darwin
            EXTRA_CMAKE_ARGS=(-DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/$PLATFORM.cmake);;
        "iOS_Simulator")
            ARCH=x86_64
            SYSROOT=`xcodebuild -version -sdk iphonesimulator Path`
            EXTRA_CMAKE_ARGS=(-DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/$PLATFORM.cmake);;
        "macOS")
            ARCH=x86_64
            SYSROOT=`xcodebuild -version -sdk macosx Path`
            EXTRA_CMAKE_ARGS=(-DCMAKE_OSX_ARCHITECTURES=$ARCH);;
    esac
}

function buildCMake() {
    rm -rf _build && mkdir _build && cd _build
    cmake -DCMAKE_INSTALL_PREFIX=$DEPS_SYSROOT -DCMAKE_PREFIX_PATH=$DEPS_SYSROOT "${EXTRA_CMAKE_ARGS[@]}" "$@" .. # >/dev/null 2>&1
    cmake --build . --target install # >/dev/null 2>&1
}

function configureThenMakeArm() {
    export CFLAGS="-isysroot $SYSROOT -arch $ARCH $CFLAGS"
    export CPPFLAGS="-isysroot $SYSROOT -arch $ARCH $CPPFLAGS"

    if [ -z "$MAKE_HOST_TRIPLE_ZLIB" ]
    then
        ./configure --prefix=$DEPS_SYSROOT "$@" # >/dev/null 2>&1
    else
        ./configure --prefix=$DEPS_SYSROOT --host=$MAKE_HOST_TRIPLE_ZLIB "$@" # >/dev/null 2>&1
    fi

    make && make install # >/dev/null 2>&1

    export -n CFLAGS
    export -n CPPFLAGS
}

function configureThenMake() {
    export CFLAGS="-isysroot $SYSROOT -arch $ARCH $CFLAGS"
    export CPPFLAGS="-isysroot $SYSROOT -arch $ARCH $CPPFLAGS"

    if [ -z "$MAKE_HOST_TRIPLE" ]
    then
        ./configure --prefix=$DEPS_SYSROOT "$@" # >/dev/null 2>&1
    else
        ./configure --prefix=$DEPS_SYSROOT --host=$MAKE_HOST_TRIPLE "$@" # >/dev/null 2>&1
    fi

    make && make install >/dev/null 2>&1

    export -n CFLAGS
    export -n CPPFLAGS
}

function buildQt5Host() {
    ./configure -prefix $QT_HOST_PREFIX -opensource -confirm-license -nomake examples -nomake tests
    make && make install
}

function buildQt5() {
    ./configure -prefix $DEPS_SYSROOT -opensource -confirm-license -nomake examples -nomake tests
    make && make install
}

function buildQt6Host() {
    ./configure -prefix $QT_HOST_PREFIX -opensource -confirm-license -release
    make && make install
}

# Build Qt https://doc.qt.io/qt-6/ios-building-from-source.html
function buildQt6() {
    ./configure -prefix $DEPS_SYSROOT -opensource -confirm-license -platform macx-ios-clang -release -qt-host-path $QT_HOST_PREFIX # -sysroot $DEPS_SYSROOT -system-zlib -system-libpng -system-freetype -pkg-config
    make && make install
}

function buildHostTools() {
    cd $SRC_PATH/pkg-config-0.29.2
    ./configure --prefix=$DEPS_SYSROOT --with-internal-glib >/dev/null 2>&1
    make && make install >/dev/null 2>&1

    #cd $SRC_PATH/autoconf-2.69
    #./configure --prefix=$DEPS_SYSROOT
    #make && make install

    #cd $SRC_PATH/automake-1.15
    #./configure --prefix=$DEPS_SYSROOT
    #make && make install

    #cd $SRC_PATH/libtool-2.4.6
    #./configure --prefix=$DEPS_SYSROOT
    #make && make install

    #cd $SRC_PATH/mm-common-1.0.4
    #./autogen.sh
    #./configure --prefix=$DEPS_SYSROOT
    #make USE_NETWORK=yes && make install
}

function main() {
    # Build zlib
    echo "Build zlib"
    cd $SRC_PATH/zlib-1.2.12
    configureThenMakeArm

    # Build TinyXML2
    echo "Build TinyXML2"
    cd $SRC_PATH/tinyxml2-9.0.0
    buildCMake

    # Build libpng (needs: zlib)
    echo "Build libpng"
    cd $SRC_PATH/libpng-1.6.37
    configureThenMake

    # Build pixman (needs: libpng)
    echo "Build pixman"
    cd $SRC_PATH/pixman-0.40.0
    configureThenMakeArm

    # Build FreeType
    echo "Build freetype"
    cd $SRC_PATH/freetype-2.12.1
    buildCMake -DFT_DISABLE_HARFBUZZ=ON -DFT_DISABLE_BZIP2=ON

    # Build cairo (needs: FreeType, pixman)
    echo "Build cairo"
    cd $SRC_PATH/cairo-1.16.0
    export PKG_CONFIG=$DEPS_SYSROOT/bin/pkg-config
    export PKG_CONFIG_PATH=$DEPS_SYSROOT/lib/pkgconfig
    configureThenMakeArm --disable-xlib --enable-svg=no --with-sysroot=$DEPS_SYSROOT CFLAGS="-I$DEPS_SYSROOT/include/freetype2" CPPFLAGS="-I$DEPS_SYSROOT/include/freetype2"

    # Build Bullet3
    echo "Build Bullet3"
    cd $SRC_PATH/bullet3-3.24
    buildCMake -DBUILD_BULLET2_DEMOS=OFF -DBUILD_OPENGL3_DEMOS=OFF -DBUILD_UNIT_TESTS=OFF

    # Build gflags
    echo "Build Bullet3"
    cd $SRC_PATH/gflags-2.2.2
    buildCMake

    # Build glog (needs: gflags)
    echo "Build Bullet3"
    cd $SRC_PATH/glog-0.6.0
    buildCMake -DWITH_GTEST=OFF -DBUILD_TESTING=OFF

    # Build GoogleTest
    echo "Build GoogleTest"
    cd $SRC_PATH/googletest-release-1.12.1
    buildCMake

    # Build ABSL
    echo "Build ABSL"
    cd $SRC_PATH/abseil-cpp-20220623.0
    buildCMake -DCMAKE_CXX_STANDARD=14

    # Build GMP
    echo "Build GMP"
    cd $SRC_PATH/gmp-6.2.1
    configureThenMake

    # Build MPFR (needs: GMP)
    echo "Build MPFR"
    cd $SRC_PATH/mpfr-4.1.0
    configureThenMake --with-gmp=$DEPS_SYSROOT

    # Build ProtoBuf
    echo "Build ProtoBuf"
    cd $SRC_PATH/protobuf-3.21.5
    buildCMake -Dprotobuf_BUILD_TESTS=OFF

    # Build Lua, Boost and Qt5 (macOS only)
    case $PLATFORM in
        "macOS")
            # Build Lua
            echo "Build Lua"
            cd $SRC_PATH/lua-5.4.4
            make macosx
            make install INSTALL_TOP=$DEPS_SYSROOT

            # Build Boost
            echo "Build Boost"
            cd $SRC_PATH/boost_1_80_0
            ./bootstrap.sh --prefix=$DEPS_SYSROOT --without-libraries=python
            ./b2 install

            # Build Qt5
            echo "Build Qt5"
            cd $SRC_PATH/qtbase-everywhere-src-5.15.5
            buildQt5;;
    esac

    # Build SuiteSparse
    echo "Build SuiteSparse"
    cd $SRC_PATH/SuiteSparse-5.12.0
    sed -i.bak 's/#define IDXTYPEWIDTH 64/#define IDXTYPEWIDTH 32/' metis-5.1.0/include/metis.h
    make library CF="-isysroot $SYSROOT -arch $ARCH -I $DEPS_SYSROOT/include" LDFLAGS="-L$DEPS_SYSROOT/lib"
    make install INSTALL=$DEPS_SYSROOT CF="-I $DEPS_SYSROOT/include" LDFLAGS="-L$DEPS_SYSROOT/lib"

    # Build Eigen3 (needs: SuiteSparse even though it is optional)
    echo "Build Eigen3"
    cd $SRC_PATH/eigen-3.4.0
    buildCMake

    # Build CERES solver (needs: gflags, glog, SuiteSparse)
    echo "Build CERES"
    cd $SRC_PATH/ceres-solver-2.0.0
    buildCMake

    # Build FLANN
    echo "Build FLANN"
    cd $SRC_PATH/flann-1.9.1
    buildCMake -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF

    # Build Point Cloud Library (PCL, needs: Boost, FLANN, Eigen3)
    echo "Build PCL"
    cd $SRC_PATH/pcl
    buildCMake
}

getAndExtractSource
setupPlatform
buildHostTools
main

cd $REPO_ROOT
tar czf deps_$PLATFORM.tar.xz deps_$PLATFORM/
