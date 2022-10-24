# Currently we assume that this script is invoked at the repo root where it resides.

# Platform to build [macOS] or [iOS, iOS_Simulator, ...]
targetPlatform=$1

# The location of this script (the repo root) where supporting files can be found
scriptDir=`pwd`

# Prefix where we built Qt for host machine
ros2HostQtPath=$scriptDir/host_deps/

# Where we download the source archives
# We download and extract at different places so that it is easier to clean up
depsDownloadPath=$scriptDir/deps_download/
mkdir -p $depsDownloadPath

# Location to extract the source
depsExtractPath=$scriptDir/deps_src/
mkdir -p $depsExtractPath

# Location to install dependencies
depsInstallPath=$scriptDir/ros2_$targetPlatform/deps

# Root for Python 3.10 to build Boost, change the match the platform such as
# pythonRoot=/Library/Frameworks/Python.framework/Versions/3.10/
# if use official Python instead of Homebrew's version on GitHub Action
pythonRoot=/usr/local/opt/python@3.10/Frameworks/Python.framework/Versions/3.10/

export PATH=$depsInstallPath/bin:$PATH
export PKG_CONFIG=$depsInstallPath/bin/pkg-config
export PKG_CONFIG_PATH=$depsInstallPath/lib/pkgconfig

platformExtraCMakeArgs=(-DCMAKE_INSTALL_PREFIX=$depsInstallPath -DCMAKE_PREFIX_PATH=$depsInstallPath)
platformBasicConfigureArgs=(--prefix=$depsInstallPath) # Configure args for regular situation
platformBasicConfigureArgsPixmanCairo=(--prefix=$depsInstallPath) # Special configure args for pixman and cairo

downloadExtract() {
    local outputFileName=$1
    local url=$2

    cd $depsDownloadPath
    curl -s -L -o $outputFileName $url

    cd $depsExtractPath
    file $depsDownloadPath/$outputFileName
    tar xzf $depsDownloadPath/$outputFileName
}

setupPlatform() {
    case $targetPlatform in
        "iOS")
            targetArch=arm64
            boostArch=arm
            targetSysroot=`xcodebuild -version -sdk iphoneos Path`
            platformBasicConfigureArgs+=(--host=aarch64-apple-darwin)
            platformBasicConfigureArgsPixmanCairo+=(--host=arm-apple-darwin) # For pixman and cairo we must use arm-apple thank to  https://gist.github.com/jvcleave/9d78de9bb27434bde2b0c3a1af355d9c
            platformExtraCMakeArgs+=(-DCMAKE_TOOLCHAIN_FILE=$scriptDir/cmake/$targetPlatform.cmake);;

        "iOS_Simulator")
            targetArch=x86_64
            boostArch=ia64
            targetSysroot=`xcodebuild -version -sdk iphonesimulator Path`
            platformExtraCMakeArgs+=(-DCMAKE_TOOLCHAIN_FILE=$scriptDir/cmake/$targetPlatform.cmake);;

        "iOS_Simulator_M1")
            targetArch=arm64
            boostArch=arm
            targetSysroot=`xcodebuild -version -sdk iphonesimulator Path`
            platformExtraCMakeArgs+=(-DCMAKE_TOOLCHAIN_FILE=$scriptDir/cmake/$targetPlatform.cmake);;

        "macCatalyst")
            targetArch=x86_64
            boostArch=ia64
            targetSysroot=`xcodebuild -version -sdk macosx Path`
            platformExtraCMakeArgs+=(-DCMAKE_TOOLCHAIN_FILE=$scriptDir/cmake/$targetPlatform.cmake);;

        "macCatalyst_M1")
            targetArch=arm64
            boostArch=arm
            targetSysroot=`xcodebuild -version -sdk macosx Path`
            platformExtraCMakeArgs+=(-DCMAKE_TOOLCHAIN_FILE=$scriptDir/cmake/$targetPlatform.cmake);;

        "macOS")
            targetArch=x86_64
            boostArch=ia64
            targetSysroot=`xcodebuild -version -sdk macosx Path`
            platformExtraCMakeArgs+=(-DCMAKE_OSX_ARCHITECTURES=$targetArch);;

        "macOS_M1")
            targetArch=arm64
            boostArch=arm
            targetSysroot=`xcodebuild -version -sdk macosx Path`
            platformBasicConfigureArgs+=(--host=aarch64-apple-darwin)
            platformBasicConfigureArgsPixmanCairo+=(--host=arm-apple-darwin)
            platformExtraCMakeArgs+=(-DCMAKE_OSX_ARCHITECTURES=$targetArch);;
    esac
}

setCompilerFlags() {
    case $targetPlatform in
        "macCatalyst")
            export CFLAGS="-isysroot $targetSysroot -target x86_64-apple-ios14.1-macabi -I$depsInstallPath/include/";;

        "macCatalyst_M1")
            export CFLAGS="-isysroot $targetSysroot -target arm64-apple-ios14.1-macabi -I$depsInstallPath/include/";;

        *)
            export CFLAGS="-isysroot $targetSysroot -arch $targetArch -I$depsInstallPath/include/";;
    esac
    export CXXFLAGS=$CFLAGS
    export CPPFLAGS=$CFLAGS # Without this, encounter error ZLIB_VERNUM != PNG_ZLIB_VERNUM when building libpng
}

buildCMake() {
    rm -rf _build && mkdir _build && cd _build
    cmake "${platformExtraCMakeArgs[@]}" "$@" .. && cmake --build . --target install # >/dev/null 2>&1 --parallel 1
}

configureThenMake() {
    setCompilerFlags
    ./configure "${platformBasicConfigureArgs[@]}" "$@" && make && make install
}

configureThenMakeArm() {
    setCompilerFlags
    ./configure "${platformBasicConfigureArgsPixmanCairo[@]}" "$@" && make && make install
}

buildHostTools() {
    downloadExtract pkg-config.tar.gz https://pkgconfig.freedesktop.org/releases/pkg-config-0.29.2.tar.gz
    cd pkg-config-0.29.2 && ./configure --prefix=$depsInstallPath --with-internal-glib && make && make install

    #downloadExtract autoconf.tar.gz http://ftpmirror.gnu.org/autoconf/autoconf-2.69.tar.gz
    #cd autoconf-2.69 && ./configure --prefix=$depsInstallPath && make && make install
    #downloadExtract automake.tar.gz http://ftpmirror.gnu.org/automake/automake-1.15.tar.gz
    #cd automake-1.15 && ./configure --prefix=$depsInstallPath && make && make install
    #downloadExtract libtool.tar.gz http://ftpmirror.gnu.org/libtool/libtool-2.4.6.tar.gz
    #cd libtool-2.4.6 && ./configure --prefix=$depsInstallPath && make && make install
    #downloadExtract mm-common.tar.gz https://github.com/GNOME/mm-common/archive/refs/tags/1.0.4.tar.gz
    #cd mm-common-1.0.4 && ./autogen.sh && ./configure --prefix=$depsInstallPath && make USE_NETWORK=yes && make install
}

buildZlib() {
    echo "Build zlib"
    downloadExtract zlib.tar.xz https://zlib.net/zlib-1.2.13.tar.xz
    # Note that zlib's configure does not set --host but relies on compiler flags environment variables
    cd zlib-1.2.13 && setCompilerFlags && ./configure --prefix=$depsInstallPath && make && make install
}

buildTinyXML2() {
    echo "Build TinyXML2"
    downloadExtract tinyxml2.tar.gz https://github.com/leethomason/tinyxml2/archive/refs/tags/9.0.0.tar.gz
    cd tinyxml2-9.0.0 && buildCMake
}

# Needs: zlib
buildLibPng() {
    echo "Build libpng"
    downloadExtract libpng.tar.xz https://download.sourceforge.net/libpng/libpng-1.6.37.tar.xz
    cd libpng-1.6.37 && configureThenMake
}

# Needs: libpng
buildPixman() {
    echo "Build pixman"
    downloadExtract pixman.tar.gz https://cairographics.org/releases/pixman-0.40.0.tar.gz
    cd pixman-0.40.0 && configureThenMakeArm
}

buildFreeType2() {
    echo "Build freetype"
    downloadExtract freetype.tar.xz https://download.savannah.gnu.org/releases/freetype/freetype-2.12.1.tar.xz
    cd freetype-2.12.1 && buildCMake -DFT_DISABLE_HARFBUZZ=ON -DFT_DISABLE_BZIP2=ON -DFT_DISABLE_ZLIB==ON -DFT_DISABLE_PNG=ON -DFT_DISABLE_BROTLI=ON
}

# Needs: FreeType, pixman
buildCairo() {
    echo "Build cairo"
    downloadExtract cairo.tar.xz https://www.cairographics.org/releases/cairo-1.16.0.tar.xz
    cd cairo-1.16.0
    sed -i.bak 's/#define HAS_DAEMON 1/#define HAS_DAEMON 0/' boilerplate/cairo-boilerplate.c
    configureThenMakeArm --disable-xlib --enable-svg=no --enable-pdf=no --enable-full-testing=no HAS_DAEMON=0
}

buildBullet3() {
    echo "Build Bullet3"
    downloadExtract bullet3.tar.gz https://github.com/bulletphysics/bullet3/archive/refs/tags/3.24.tar.gz
    cd bullet3-3.24 && buildCMake -DBUILD_BULLET2_DEMOS=OFF -DBUILD_OPENGL3_DEMOS=OFF -DBUILD_UNIT_TESTS=OFF
}

buildGFlags() {
    echo "Build gflags"
    downloadExtract gflags.tar.gz https://github.com/gflags/gflags/archive/refs/tags/v2.2.2.tar.gz
    cd gflags-2.2.2 && buildCMake
}

# Needs: gflags
buildGlog() {
    echo "Build glog"
    downloadExtract glog.tar.gz https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz
    cd glog-0.6.0 && buildCMake -DWITH_GTEST=OFF -DBUILD_TESTING=OFF
}

buildGtest() {
    echo "Build GoogleTest"
    downloadExtract googletest.tar.gz https://github.com/google/googletest/archive/refs/tags/release-1.12.1.tar.gz
    cd googletest-release-1.12.1 && buildCMake
}

buildAbsl() {
    echo "Build ABSL"
    downloadExtract abseil-cpp.tar.gz https://github.com/abseil/abseil-cpp/archive/refs/tags/20220623.0.tar.gz
    cd abseil-cpp-20220623.0 && buildCMake -DCMAKE_CXX_STANDARD=14
}

buildGmp() {
    echo "Build GMP"
    downloadExtract gmp.tar.xz https://gmplib.org/download/gmp/gmp-6.2.1.tar.xz
    cd gmp-6.2.1 && configureThenMake
}

# Needs: GMP
buildMpfr() {
    echo "Build MPFR"
    downloadExtract mpfr.tar.xz https://www.mpfr.org/mpfr-current/mpfr-4.1.0.tar.xz
    cd mpfr-4.1.0 && configureThenMake --with-gmp=$depsInstallPath
}

buildProtoBuf() {
    echo "Build ProtoBuf"
    downloadExtract protobuf.tar.gz https://github.com/protocolbuffers/protobuf/releases/download/v21.5/protobuf-cpp-3.21.5.tar.gz
    cd protobuf-3.21.5 && buildCMake -Dprotobuf_BUILD_TESTS=OFF
}

buildLua() {
    echo "Build Lua"
    downloadExtract lua.tar.gz https://www.lua.org/ftp/lua-5.4.4.tar.gz
    cd lua-5.4.4 && make macosx && make install INSTALL_TOP=$depsInstallPath
}

buildBoost() {
    echo "Build Boost"
    downloadExtract boost.tar.gz https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.gz
    cd boost_1_80_0

    # Note: We must pass the full path to the python3 executable in --with-python-root=$pythonRoot/bin/python3
    # otherwise, will run into the issue https://github.com/boostorg/boost/issues/693.
    # Searching the entire Boost source code for the string `python-cfg` reveals a single file
    #     tools/build/src/tools/python.jam
    # and in that file, the function-like `local rule configure`'s parameter `$cmd-or-prefix` appears to be initialized
    # with whatever passed to `--with-python-root` as it tried to find Python. Since it only tries `bin/python`,
    # it will never succeed on Mac because bin/python does not exist.
    ./bootstrap.sh --prefix=$depsInstallPath \
                   --with-python=$pythonRoot/bin/python3 \
                   --with-python-version=3.10 \
                   --with-python-root=$pythonRoot/bin/python3 \
                   --without-libraries=coroutine

    ./b2 install architecture=$boostArch # --debug-configuration --debug-building --debug-generator -d+2
}

buildQt5() {
    echo "Build Qt5"
    downloadExtract qtbase.tar.gz https://download.qt.io/archive/qt/5.15/5.15.5/submodules/qtbase-everywhere-opensource-src-5.15.5.tar.xz
    cd qtbase-everywhere-src-5.15.5

    # Patch the source https://codereview.qt-project.org/c/qt/qtbase/+/378706
    sed -i.bak "s,QT_BEGIN_NAMESPACE,#include <CoreGraphics/CGColorSpace.h>\n#include <IOSurface/IOSurface.h>\nQT_BEGIN_NAMESPACE," src/plugins/platforms/cocoa/qiosurfacegraphicsbuffer.h

    ./configure -prefix $depsInstallPath -opensource -confirm-license -nomake examples -nomake tests -no-framework -device-option QMAKE_APPLE_DEVICE_ARCHS=$targetArch
    make && make install
}

buildOpenCV() {
    echo "Build OpenCV"
    downloadExtract opencv.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.6.0.tar.gz
    # https://docs.opencv.org/4.x/db/d05/tutorial_config_reference.html
    cd opencv-4.6.0 && buildCMake -DCMAKE_BUILD_TYPE=Release -DBUILD_opencv_python2=OFF -DBUILD_JAVA=OFF -DBUILD_OBJC=OFF -DBUILD_ZLIB=NO -DBUILD_OPENEXR=YES
}

buildSuiteSparse() {
    echo "Build SuiteSparse"
    downloadExtract SuiteSparse.tar.gz https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/refs/tags/v5.12.0.tar.gz
    cd SuiteSparse-5.12.0
    setCompilerFlags

    # By default, SuiteParse uses 64-bit integer for its `idx_t` (`long long`).
    # This unfortunately clashes with Eigen3's usage of `int`.
    # So change this in the included `SuiteParse/metis-5.1.0/include/metis.h`.
    sed -i.bak 's/#define IDXTYPEWIDTH 64/#define IDXTYPEWIDTH 32/' metis-5.1.0/include/metis.h

    #export CMAKE_OPTIONS="-DCMAKE_INSTALL_PREFIX=$depsInstallPath -DCMAKE_TOOLCHAIN_FILE=$scriptDir/cmake/$targetPlatform.cmake"
    #sed -i.bak 's;^CONFIG_FLAGS = ;CONFIG_FLAGS = -DCMAKE_TOOLCHAIN_FILE=\\$(prefix)/../../cmake/$targetPlatform.cmake;' metis-5.1.0/Makefile
    #sed -i.bak 's/^return system.*$/return 1;/' metis-5.1.0/GKlib/fs.c

    make static
    make install INSTALL=$depsInstallPath
}

# Needs: SuiteSparse for cartographer; but optional for rviz2
buildEigen3() {
    echo "Build Eigen3"
    downloadExtract eigen.tar.bz2 https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.bz2
    cd eigen-3.4.0 && buildCMake
}

# Needs: gflags, glog, SuiteSparse
buildCERES() {
    echo "Build CERES"
    # For CERES, must checkout `2.0.0` to avoid https://github.com/cartographer-project/cartographer/issues/1879
    downloadExtract ceres-solver.tar.gz http://ceres-solver.org/ceres-solver-2.0.0.tar.gz
    cd ceres-solver-2.0.0 && buildCMake
}

buildFLANN() {
    echo "Build FLANN"
    downloadExtract flann.tar.gz https://github.com/flann-lib/flann/archive/refs/tags/1.9.1.tar.gz
    cd flann-1.9.1 && buildCMake -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
}

# Needs: Boost, FLANN, Eigen3
buildPCL() {
    echo "Build PCL"
    downloadExtract pcl.tar.gz https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.12.1/source.tar.gz
    cd pcl && buildCMake
}

buildAll() {
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
    buildPCL
}

buildMoveItDeps() {
    downloadExtract ccd.tar.gz https://github.com/danfis/libccd/archive/refs/tags/v2.1.tar.gz
    cd libccd-2.1 && buildCMake

    downloadExtract octomap.tar.gz https://github.com/OctoMap/octomap/archive/refs/tags/v1.9.6.tar.gz
    cd octomap-1.9.6 && buildCMake

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

    # FCL depends on octomap, libccd, Eigen3
    downloadExtract fcl.tar.gz https://github.com/flexible-collision-library/fcl/archive/refs/tags/0.7.0.tar.gz
    cd fcl-0.7.0 && buildCMake -DBUILD_TESTING=NO

    downloadExtract qhull.tgz http://www.qhull.org/download/qhull-2020-src-8.0.2.tgz
    cd qhull-2020.2 && buildCMake

    downloadExtract assimp.tar.gz https://github.com/assimp/assimp/archive/refs/tags/v5.2.5.tar.gz
    cd assimp-5.2.5 && buildCMake

    downloadExtract ruckig.tar.gz https://github.com/pantor/ruckig/archive/refs/tags/v0.8.4.tar.gz
    cd ruckig-0.8.4 && buildCMake

    downloadExtract glew.tgz https://github.com/nigels-com/glew/releases/download/glew-2.2.0/glew-2.2.0.tgz
    cd glew-2.2.0/build/cmake && buildCMake

    # FreeGLUT needs X11 (provided by XQuartz for macOS)
    # For some reason the X11 include path is not added so we force add it
    downloadExtract freeglut.tar.gz https://github.com/FreeGLUTProject/freeglut/releases/download/v3.4.0/freeglut-3.4.0.tar.gz
    cd freeglut-3.4.0 && buildCMake -DFREEGLUT_BUILD_DEMOS=NO -DCMAKE_C_FLAGS="-isystem /usr/X11R6/include"

    downloadExtract openssl.tar.gz https://github.com/openssl/openssl/archive/refs/tags/openssl-3.0.6.tar.gz
    cd openssl-openssl-3.0.6 && ./Configure --prefix=$depsInstallPath && make && make install

    # Needs Boost
    downloadExtract omplcore.tar.gz https://github.com/ompl/ompl/archive/1.5.2.tar.gz
    cd ompl-1.5.2 && buildCMake

    downloadExtract openmp.tar.xz https://github.com/llvm/llvm-project/releases/download/llvmorg-14.0.6/openmp-14.0.6.src.tar.xz
    cd openmp-14.0.6.src && buildCMake
}

buildAllDeps() {
    setupPlatform
    buildHostTools

    case $targetPlatform in
        "macOS"|"macOS_M1") # Build dependencies for RVIZ and OpenCV
            buildZlib
            buildLibPng
            buildFreeType2
            buildEigen3
            buildTinyXML2
            buildBullet3
            buildQt5
            buildBoost
            buildOpenCV
            buildMoveItDeps;;

        *) # Build useful dependencies for iOS
            buildTinyXML2;;
    esac
}

buildCollection() {
    # Dependencies collection should be [core, boost, opencv, ...]
    collection=$1

    setupPlatform

    case $collection in
        "core")
            buildHostTools
            buildZlib
            buildLibPng;;

        "boost")
            buildBoost;;

        "qt5")
            buildQt5;;

        "opencv")
            buildOpenCV;;

        "rviz")
            buildFreeType2
            buildEigen3
            buildTinyXML2
            buildBullet3;;

        "moveit")
            buildMoveItDeps;;
    esac
}
