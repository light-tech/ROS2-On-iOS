SET(CMAKE_SYSTEM_NAME Darwin)
SET(CMAKE_SYSTEM_VERSION 13)
SET(CMAKE_CXX_COMPILER_WORKS True)
SET(CMAKE_C_COMPILER_WORKS True)

execute_process(COMMAND xcodebuild -version -sdk macosx Path
                OUTPUT_VARIABLE CMAKE_OSX_SYSROOT
                ERROR_QUIET
                OUTPUT_STRIP_TRAILING_WHITESPACE)

SET(CMAKE_OSX_ARCHITECTURES "arm64")
SET(CMAKE_CROSSCOMPILING True)
SET(CMAKE_SYSTEM_PROCESSOR "arm64")
