SET(CMAKE_SYSTEM_NAME Darwin)
SET(CMAKE_SYSTEM_VERSION 13)
SET(CMAKE_CXX_COMPILER_WORKS True)
SET(CMAKE_C_COMPILER_WORKS True)
SET(IOS True)

execute_process(COMMAND xcodebuild -version -sdk iphonesimulator Path
                OUTPUT_VARIABLE CMAKE_OSX_SYSROOT
                ERROR_QUIET
                OUTPUT_STRIP_TRAILING_WHITESPACE)

SET(CMAKE_C_FLAGS "-target x86_64-apple-ios-simulator -mios-version-min=14.0")
SET(CMAKE_CXX_FLAGS "-target x86_64-apple-ios-simulator -mios-version-min=14.0")
SET(CMAKE_CROSSCOMPILING True)
SET(CMAKE_SYSTEM_PROCESSOR "x86_64")
