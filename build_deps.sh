# Platform to build [macOS] or [iOS, iOS_Simulator, ...]
targetPlatform=$1

# This repo root (where this script is located)
REPO_ROOT=`pwd`

# The sysroot of the dependencies we built
ros2SystemDependenciesPath=$REPO_ROOT/ros2_deps_$targetPlatform

# Prefix where we built Qt for host machine
ros2HostQtPath=$REPO_ROOT/host_deps/

# Where we download the source archives
ros2DependenciesSourceDownloadPath=$REPO_ROOT/deps_download/

# Location to extract the source
ros2DependenciesSourceExtractionPath=$REPO_ROOT/deps_src/

export PATH=$ros2SystemDependenciesPath/bin:$PATH
export PKG_CONFIG=$ros2SystemDependenciesPath/bin/pkg-config
export PKG_CONFIG_PATH=$ros2SystemDependenciesPath/lib/pkgconfig

function getSource() {
    mkdir -p $ros2DependenciesSourceDownloadPath
    cd $ros2DependenciesSourceDownloadPath

    curl -s -L -o pkg-config.tar.gz https://pkgconfig.freedesktop.org/releases/pkg-config-0.29.2.tar.gz
         #-o autoconf.tar.gz http://ftpmirror.gnu.org/autoconf/autoconf-2.69.tar.gz \
         #-o automake.tar.gz http://ftpmirror.gnu.org/automake/automake-1.15.tar.gz \
         #-o libtool.tar.gz http://ftpmirror.gnu.org/libtool/libtool-2.4.6.tar.gz \
         #-o mm-common.tar.gz https://github.com/GNOME/mm-common/archive/refs/tags/1.0.4.tar.gz

    # Dependencies for rviz
    # Need -L to download github releases according to https://stackoverflow.com/questions/46060010/download-github-release-with-curl
    curl -s -L -o freetype.tar.xz https://download.savannah.gnu.org/releases/freetype/freetype-2.12.1.tar.xz \
         -o libpng.tar.xz https://download.sourceforge.net/libpng/libpng-1.6.37.tar.xz \
         -o zlib.tar.xz https://zlib.net/zlib-1.2.12.tar.xz \
         -o eigen.tar.bz2 https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.bz2 \
         -o tinyxml2.tar.gz https://github.com/leethomason/tinyxml2/archive/refs/tags/9.0.0.tar.gz \
         -o bullet3.tar.gz https://github.com/bulletphysics/bullet3/archive/refs/tags/3.24.tar.gz \
         -o qtbase.tar.gz https://download.qt.io/archive/qt/5.15/5.15.5/submodules/qtbase-everywhere-opensource-src-5.15.5.tar.xz

    # Common heavy dependencies OpenCV, Boost
    curl -s -L -o opencv.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.6.0.tar.gz \
         -o boost.tar.gz https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.gz

    # Dependencies for cartographer
    if [ $targetPlatform != "macOS" ]; then
    curl -s -L -o abseil-cpp.tar.gz https://github.com/abseil/abseil-cpp/archive/refs/tags/20220623.0.tar.gz \
         -o gflags.tar.gz https://github.com/gflags/gflags/archive/refs/tags/v2.2.2.tar.gz \
         -o cairo.tar.xz https://www.cairographics.org/releases/cairo-1.16.0.tar.xz \
         -o pixman.tar.gz https://cairographics.org/releases/pixman-0.40.0.tar.gz \
         -o glog.tar.gz https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz \
         -o gmp.tar.xz https://gmplib.org/download/gmp/gmp-6.2.1.tar.xz \
         -o protobuf.tar.gz https://github.com/protocolbuffers/protobuf/releases/download/v21.5/protobuf-cpp-3.21.5.tar.gz \
         -o lua.tar.gz https://www.lua.org/ftp/lua-5.4.4.tar.gz \
         -o flann.tar.gz https://github.com/flann-lib/flann/archive/refs/tags/1.9.1.tar.gz \
         -o mpfr.tar.xz https://www.mpfr.org/mpfr-current/mpfr-4.1.0.tar.xz \
         -o pcl.tar.gz https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.12.1/source.tar.gz \
         -o googletest.tar.gz https://github.com/google/googletest/archive/refs/tags/release-1.12.1.tar.gz \
         -o SuiteSparse.tar.gz https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/refs/tags/v5.12.0.tar.gz \
         -o ceres-solver.tar.gz http://ceres-solver.org/ceres-solver-2.0.0.tar.gz
    fi
}

function extractSource() {
    mkdir -p $ros2DependenciesSourceExtractionPath
    cd $ros2DependenciesSourceExtractionPath
    local src_files=`ls $ros2DependenciesSourceDownloadPath`
    ls -all $ros2DependenciesSourceDownloadPath
    for f in $src_files; do
        tar xzf $ros2DependenciesSourceDownloadPath/$f
    done
}

function setupPlatform() {
    case $targetPlatform in
        "iOS")
            targetArch=arm64
            targetSysroot=`xcodebuild -version -sdk iphoneos Path`
            targetHostTripleForPixmanAndCairo=arm-apple-darwin # For pixman and cairo we must use arm-apple and not arm64-apple thank to  https://gist.github.com/jvcleave/9d78de9bb27434bde2b0c3a1af355d9c
            targetHostTriple=aarch64-apple-darwin
            platformExtraCMakeArgs=(-DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/$targetPlatform.cmake);;
        "iOS_Simulator")
            target=x86_64
            targetSysroot=`xcodebuild -version -sdk iphonesimulator Path`
            platformExtraCMakeArgs=(-DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/$targetPlatform.cmake);;
        "macOS")
            targetArch=x86_64
            targetSysroot=`xcodebuild -version -sdk macosx Path`
            platformExtraCMakeArgs=(-DCMAKE_OSX_ARCHITECTURES=$targetArch);;
    esac
}

function setCompilerFlags() {
    export CFLAGS="-isysroot $targetSysroot -arch $targetArch -I$ros2SystemDependenciesPath/include/"
    export CXXFLAGS=$CFLAGS
    export CPPFLAGS=$CFLAGS # Without this, encounter error ZLIB_VERNUM != PNG_ZLIB_VERNUM when building libpng
}

function buildCMake() {
    rm -rf _build && mkdir _build && cd _build
    cmake -DCMAKE_INSTALL_PREFIX=$ros2SystemDependenciesPath -DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath "${platformExtraCMakeArgs[@]}" "$@" .. # >/dev/null 2>&1
    cmake --build . --target install # >/dev/null 2>&1 --parallel 1
}

function configureThenMakeArm() {
    setCompilerFlags

    if [ -z "$targetHostTripleForPixmanAndCairo" ]
    then
        ./configure --prefix=$ros2SystemDependenciesPath "$@" # >/dev/null 2>&1
    else
        ./configure --prefix=$ros2SystemDependenciesPath --host=$targetHostTripleForPixmanAndCairo "$@" # >/dev/null 2>&1
    fi

    make && make install # >/dev/null 2>&1

    export -n CFLAGS
    export -n CXXFLAGS
}

function configureThenMake() {
    setCompilerFlags

    if [ -z "$targetHostTriple" ]
    then
        ./configure --prefix=$ros2SystemDependenciesPath "$@" # >/dev/null 2>&1
    else
        ./configure --prefix=$ros2SystemDependenciesPath --host=$targetHostTriple "$@" # >/dev/null 2>&1
    fi

    make && make install #>/dev/null 2>&1

    export -n CFLAGS
    export -n CXXFLAGS
}

function configureThenMakeNoHost() {
    setCompilerFlags

    ./configure --prefix=$ros2SystemDependenciesPath "$@" # >/dev/null 2>&1
    make && make install #>/dev/null 2>&1

    export -n CFLAGS
    export -n CXXFLAGS
}

function buildHostTools() {
    cd $ros2DependenciesSourceExtractionPath/pkg-config-0.29.2
    ./configure --prefix=$ros2SystemDependenciesPath --with-internal-glib >/dev/null 2>&1
    make && make install >/dev/null 2>&1

    #cd $ros2DependenciesSourceExtractionPath/autoconf-2.69
    #./configure --prefix=$ros2SystemDependenciesPath
    #make && make install

    #cd $ros2DependenciesSourceExtractionPath/automake-1.15
    #./configure --prefix=$ros2SystemDependenciesPath
    #make && make install

    #cd $ros2DependenciesSourceExtractionPath/libtool-2.4.6
    #./configure --prefix=$ros2SystemDependenciesPath
    #make && make install

    #cd $ros2DependenciesSourceExtractionPath/mm-common-1.0.4
    #./autogen.sh
    #./configure --prefix=$ros2SystemDependenciesPath
    #make USE_NETWORK=yes && make install
}

function buildZlib() {
    echo "Build zlib"
    cd $ros2DependenciesSourceExtractionPath/zlib-1.2.12
    configureThenMakeNoHost # Note that zlib's configure does not set --host
}

function buildTinyXML2() {
    echo "Build TinyXML2"
    cd $ros2DependenciesSourceExtractionPath/tinyxml2-9.0.0
    buildCMake
}

# Needs: zlib
function buildLibPng() {
    echo "Build libpng"
    cd $ros2DependenciesSourceExtractionPath/libpng-1.6.37
    configureThenMake
}

# Needs: libpng
function buildPixman() {
    echo "Build pixman"
    cd $ros2DependenciesSourceExtractionPath/pixman-0.40.0
    configureThenMakeArm
}

function buildFreeType2() {
    echo "Build freetype"
    cd $ros2DependenciesSourceExtractionPath/freetype-2.12.1
    buildCMake -DFT_DISABLE_HARFBUZZ=ON -DFT_DISABLE_BZIP2=ON -DFT_DISABLE_ZLIB==ON -DFT_DISABLE_PNG=ON -DFT_DISABLE_BROTLI=ON
}

# Needs: FreeType, pixman
function buildCairo() {
    echo "Build cairo"
    cd $ros2DependenciesSourceExtractionPath/cairo-1.16.0
    sed -i.bak 's/#define HAS_DAEMON 1/#define HAS_DAEMON 0/' boilerplate/cairo-boilerplate.c
    configureThenMakeArm --disable-xlib --enable-svg=no --enable-pdf=no --enable-full-testing=no HAS_DAEMON=0
}

function buildBullet3() {
    echo "Build Bullet3"
    cd $ros2DependenciesSourceExtractionPath/bullet3-3.24
    buildCMake -DBUILD_BULLET2_DEMOS=OFF -DBUILD_OPENGL3_DEMOS=OFF -DBUILD_UNIT_TESTS=OFF
}

function buildGFlags() {
    echo "Build gflags"
    cd $ros2DependenciesSourceExtractionPath/gflags-2.2.2
    buildCMake
}

# Needs: gflags
function buildGlog() {
    echo "Build glog"
    cd $ros2DependenciesSourceExtractionPath/glog-0.6.0
    buildCMake -DWITH_GTEST=OFF -DBUILD_TESTING=OFF
}

function buildGtest() {
    echo "Build GoogleTest"
    cd $ros2DependenciesSourceExtractionPath/googletest-release-1.12.1
    buildCMake
}

function buildAbsl() {
    echo "Build ABSL"
    cd $ros2DependenciesSourceExtractionPath/abseil-cpp-20220623.0
    buildCMake -DCMAKE_CXX_STANDARD=14
}

function buildGmp() {
    echo "Build GMP"
    cd $ros2DependenciesSourceExtractionPath/gmp-6.2.1
    configureThenMake
}

# Needs: GMP
function buildMpfr() {
    echo "Build MPFR"
    cd $ros2DependenciesSourceExtractionPath/mpfr-4.1.0
    configureThenMake --with-gmp=$ros2SystemDependenciesPath
}

function buildProtoBuf() {
    echo "Build ProtoBuf"
    cd $ros2DependenciesSourceExtractionPath/protobuf-3.21.5
    buildCMake -Dprotobuf_BUILD_TESTS=OFF
}

function buildLua() {
    echo "Build Lua"
    cd $ros2DependenciesSourceExtractionPath/lua-5.4.4
    make macosx
    make install INSTALL_TOP=$ros2SystemDependenciesPath
}

function buildBoost() {
    echo "Build Boost"
    cd $ros2DependenciesSourceExtractionPath/boost_1_80_0
    ./bootstrap.sh --prefix=$ros2SystemDependenciesPath --without-libraries=python
    ./b2 install
}

function buildQt5Host() {
    echo "Build Qt5 for Build machine"
    cd $ros2DependenciesSourceExtractionPath/qtbase-everywhere-src-5.15.5
    ./configure -prefix $ros2HostQtPath -opensource -confirm-license -nomake examples -nomake tests -no-framework
    make && make install
}

function buildQt5() {
    echo "Build Qt5 for Host target"
    cd $ros2DependenciesSourceExtractionPath/qtbase-everywhere-src-5.15.5

    # Patch the source https://codereview.qt-project.org/c/qt/qtbase/+/378706
    sed -i.bak "s,QT_BEGIN_NAMESPACE,#include <CoreGraphics/CGColorSpace.h>\n#include <IOSurface/IOSurface.h>\nQT_BEGIN_NAMESPACE," src/plugins/platforms/cocoa/qiosurfacegraphicsbuffer.h

    ./configure -prefix $ros2SystemDependenciesPath -opensource -confirm-license -nomake examples -nomake tests -no-framework
    make && make install
}

function buildQt6Host() {
    ./configure -prefix $ros2HostQtPath -opensource -confirm-license -release
    make && make install
}

# Build Qt https://doc.qt.io/qt-6/ios-building-from-source.html
function buildQt6() {
    ./configure -prefix $ros2SystemDependenciesPath -opensource -confirm-license -platform macx-ios-clang -release -qt-host-path $ros2HostQtPath # -sysroot $ros2SystemDependenciesPath -system-zlib -system-libpng -system-freetype -pkg-config
    make && make install
}

function buildSuiteSparse() {
    echo "Build SuiteSparse"
    cd $ros2DependenciesSourceExtractionPath/SuiteSparse-5.12.0
    setCompilerFlags
    export CMAKE_OPTIONS="-DCMAKE_INSTALL_PREFIX=$ros2SystemDependenciesPath -DCMAKE_TOOLCHAIN_FILE=$REPO_ROOT/cmake/$targetPlatform.cmake"
    #echo $CMAKE_OPTIONS
    #sed -i.bak 's/#define IDXTYPEWIDTH 64/#define IDXTYPEWIDTH 32/' metis-5.1.0/include/metis.h
    #sed -i.bak 's;^CONFIG_FLAGS = ;CONFIG_FLAGS = -DCMAKE_TOOLCHAIN_FILE=\\$(prefix)/../../cmake/$targetPlatform.cmake;' metis-5.1.0/Makefile
    #sed -i.bak 's/^return system.*$/return 1;/' metis-5.1.0/GKlib/fs.c
    make static
    make install INSTALL=$ros2SystemDependenciesPath
}

# Needs: SuiteSparse even though it is optional
function buildEigen3() {
    echo "Build Eigen3"
    cd $ros2DependenciesSourceExtractionPath/eigen-3.4.0
    buildCMake
}

# Needs: gflags, glog, SuiteSparse
function buildCERES() {
    echo "Build CERES"
    cd $ros2DependenciesSourceExtractionPath/ceres-solver-2.0.0
    buildCMake
}

function buildFLANN() {
    echo "Build FLANN"
    cd $ros2DependenciesSourceExtractionPath/flann-1.9.1
    buildCMake -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
}

# Needs: Boost, FLANN, Eigen3
function buildPCL() {
    echo "Build PCL"
    cd $ros2DependenciesSourceExtractionPath/pcl
    buildCMake
}

function buildOpenCV() {
    echo "Build OpenCV"
    cd $ros2DependenciesSourceExtractionPath/opencv-4.6.0
    buildCMake -DCMAKE_BUILD_TYPE=Release -DBUILD_opencv_python2=OFF -DBUILD_JAVA=OFF -DBUILD_OBJC=OFF -DBUILD_ZLIB=NO
}


getSource
extractSource
setupPlatform
buildHostTools

case $targetPlatform in
    "macOS") # Build dependencies for RVIZ and OpenCV
        buildZlib
        buildLibPng
        buildFreeType2
        buildEigen3
        buildTinyXML2
        buildBullet3
        buildQt5
        buildBoost
        buildOpenCV;;

    *) # Build dependencies for ROS2 cartographer package
        buildZlib
        buildTinyXML2
        buildLibPng
        buildPixman
        buildFreeType2
        buildCairo
        buildBullet3
        buildGFlags
        buildGlog
        buildGtest
        buildAbsl
        buildGmp
        buildMpfr
        buildProtoBuf

        # Build Lua, Boost and Qt5 (macOS only)
        #case $targetPlatform in
        #    "macOS")
        #        buildLua
        #        buildBoost
        #        buildQt5;;
        #esac

        buildSuiteSparse
        buildEigen3
        buildCERES
        buildFLANN
        buildPCL;;
esac

cd $REPO_ROOT
