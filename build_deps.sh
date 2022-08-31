# This repo root (where this script is located)
REPO_ROOT=`pwd`

# The sysroot of the dependencies we built
DEPS_SYSROOT=$REPO_ROOT/deps

# Prefix where we built Qt for host machine
QT_HOST_PREFIX=$REPO_ROOT/host_deps/

# Where we download the source archives
DOWNLOAD_PATH=$REPO_ROOT/deps/download/

# Location to extract the source
SRC_PATH=$REPO_ROOT/deps/src/

function getSource() {
    mkdir -p $DOWNLOAD_PATH
    cd $DOWNLOAD_PATH
    # Need -L to download github releases according to https://stackoverflow.com/questions/46060010/download-github-release-with-curl
    curl -s -L -o abseil-cpp.tar.gz https://github.com/abseil/abseil-cpp/archive/refs/tags/20220623.0.tar.gz \
         -o boost.tar.gz https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.gz \
         -o gflags.tar.gz https://github.com/gflags/gflags/archive/refs/tags/v2.2.2.tar.gz \
         -o cairo.tar.xz https://cairographics.org/releases/cairomm-1.16.0.tar.xz \
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
         -o ceres-solver.tar.gz http://ceres-solver.org/ceres-solver-2.0.0.tar.gz
}

function extractSource() {
    mkdir -p $SRC_PATH
    cd $SRC_PATH
    local src_files=`ls $DOWNLOAD_PATH`
    for f in $src_files; do
        echo "Extract " $f
        tar xzf $DOWNLOAD_PATH/$f
    done
}

function setupPlatform() {
    EXTRA_CMAKE_ARGS=()
    EXTRA_CONFIGURE_ARGS=()
}

function buildCMake() {
    mkdir _build && cd _build
    cmake -DCMAKE_INSTALL_PREFIX=$DEPS_SYSROOT -DCMAKE_PREFIX_PATH=$DEPS_SYSROOT "$@" ..
    cmake --build . --target install
}

function configureThenMake() {
    ./configure --prefix=$DEPS_SYSROOT "$@"
    make && make install
}

function buildQt5Host() {
    ./configure -prefix $QT_HOST_PREFIX -opensource -confirm-license
    make && make install
}

function buildQt6Host() {
    ./configure -prefix $QT_HOST_PREFIX -opensource -confirm-license -release
    make && make install
}

# Build Qt https://doc.qt.io/qt-6/ios-building-from-source.html
function buildQt6() {
    ./configure -prefix $DEPS_SYSROOT -opensource -confirm-license -platform macx-ios-clang -release -qt-host-path $QT_HOST_PREFIX #  -sysroot $DEPS_SYSROOT -system-zlib -system-libpng -system-freetype -pkg-config
    make && make install
}

function main() {
    # Build pkg-config for HOST
    cd $SRC_PATH/pkg-config-0.29.2
    configureThenMake --with-internal-glib

    # Build zlib
    cd $SRC_PATH/zlib-1.2.12
    configureThenMake

    # Build TinyXML2
    cd $SRC_PATH/tinyxml2-9.0.0
    buildCMake

    # Build libpng (needs: zlib)
    cd $SRC_PATH/libpng-1.6.37
    configureThenMake

    # Build pixman (needs: libpng)
    cd $SRC_PATH/pixman-0.40.0
    configureThenMake

    # Build FreeType
    cd $SRC_PATH/freetype-2.12.1
    buildCMake -DFT_DISABLE_HARFBUZZ=ON -DFT_DISABLE_BZIP2=ON

    # Build cairo (needs: FreeType, pixman)
    cd $SRC_PATH/cairo-1.16.0
    export PKG_CONFIG=$DEPS_SYSROOT/bin/pkg-config
    export PKG_CONFIG_PATH=$DEPS_SYSROOT/lib/pkgconfig
    configureThenMake --disable-xlib --enable-svg=no --with-sysroot=$REPO_ROOT/deps CFLAGS="-I$DEPS_SYSROOT/include/freetype2" CPPFLAGS="-I$DEPS_SYSROOT/include/freetype2"

    # Build Bullet3
    cd $SRC_PATH/bullet3-3.24
    buildCMake

    # Build Eigen3 (might depend on Qt)
    cd $SRC_PATH/eigen-3.4.0
    buildCMake

    # Build Qt5
    #cd $SRC_PATH/qtbase-everywhere-src-5.15.5
    #buildQt5Host

    # Build Qt6
    #cd $SRC_PATH/qtbase-everywhere-src-6.3.1
    #buildQt6Host
    #buildQt6

    # Build gflags
    cd $SRC_PATH/gflags-2.2.2
    buildCMake

    # Build glog (needs: gflags)
    cd $SRC_PATH/glog-0.6.0
    buildCMake -DWITH_GTEST=OFF

    # Build GoogleTest
    cd $SRC_PATH/googletest-release-1.12.1
    buildCMake

    # Build ABSL
    cd $SRC_PATH/abseil-cpp-20220623.0
    buildCMake -DCMAKE_CXX_STANDARD=14

    # Build GMP
    cd $SRC_PATH/gmp-6.2.1
    configureThenMake

    # Build MPFR (needs: GMP)
    cd $SRC_PATH/mpfr-4.1.0
    configureThenMake --with-gmp=$DEPS_SYSROOT

    # Build ProtoBuf
    cd $SRC_PATH/protobuf-3.21.5
    buildCMake -Dprotobuf_BUILD_TESTS=OFF

    # Build Lua
    cd $SRC_PATH/lua-5.4.4
    make macosx
    make install INSTALL_TOP=$DEPS_SYSROOT

    # Build Boost
    cd $SRC_PATH/boost_1_80_0
    ./bootstrap.sh --prefix=$DEPS_SYSROOT --without-libraries=python
    ./b2 install

    # Build SuiteSparse
    cd $SRC_PATH/SuiteSparse-5.12.0
    sed -i.bak 's/#define IDXTYPEWIDTH 64/#define IDXTYPEWIDTH 32/' metis-5.1.0/include/metis.h
    make
    make install INSTALL=$DEPS_SYSROOT CF="-I $DEPS_SYSROOT/include" LDFLAGS="-L$DEPS_SYSROOT/lib"

    # Build CERES solver (needs: gflags, glog, SuiteSparse)
    cd $SRC_PATH/ceres-solver-2.0.0
    buildCMake

    # Build FLANN
    cd $SRC_PATH/flann-1.9.1
    buildCMake -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF

    # Build Point Cloud Library (PCL, needs: Boost, FLANN, Eigen3)
    cd $SRC_PATH/pcl
    buildCMake
}

getSource
extractSource
main
cd $REPO_ROOT
tar czf deps.tar.xz deps/
