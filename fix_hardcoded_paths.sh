read -p "Where did you extract ROS2 to (e.g. $HOME/usr/ros2)? " installDir
echo

read -p "Where is your virtual Python environment (e.g. $HOME/usr/ros2PythonEnv)? " localPythonEnvPath
echo

read -p "Can you locate the path to libpython3.10.dylib (e.g. /Library/Frameworks/Python.framework/Versions/3.10/lib/python3.10/config-3.10-darwin/libpython3.10.dylib if you install Python using the official installer at python.org)? " localLibPythonPath
echo

read -p "Confirm replacement? [y/n]" confirmBeforeAction
echo

# Helper function to confirm before doing a command
#
# Main usage:      confirm PROMPT && COMMAND
#
# The effect is to ask the user with the given PROMPT message [should be y/n] to confirm the action
# and then execute COMMAND if the user answers 'y'.
confirm() {
    if [ "$confirmBeforeAction" == "n" ]; then
        return 0
    fi

    local proceed="n"
    read -p "$1" proceed
    case $proceed in
        "y")
            return 0;;
        *)
            return -1;;
    esac
}

fixHardCodedPythonEnvPath() {
    local hardcodedPythonEnvPath=/Users/runner/work/ROS2-On-iOS/ROS2-On-iOS/ros2PythonEnv
    local hardcodedFiles=($(grep -r -l --include="*.cmake" "$hardcodedPythonEnvPath" .))

    echo "Replacing $hardcodedPythonEnvPath -> $localPythonEnvPath"

    for f in "${hardcodedFiles[@]}"; do
        confirm "In $f [y/n]? " && sed -i '' "s,$hardcodedPythonEnvPath,$localPythonEnvPath,g" $f
    done
}

fixHardCodedLibPythonPath() {
    local hardcodedLibPythonPath=/usr/local/opt/python@3.10/Frameworks/Python.framework/Versions/3.10/lib/python3.10/config-3.10-darwin/libpython3.10.dylib
    local hardcodedFiles=($(grep -r -l --include="*.cmake" "$hardcodedLibPythonPath" .))

    echo "Replacing $hardcodedLibPythonPath -> $localLibPythonPath"

    for f in "${hardcodedFiles[@]}"; do
        confirm "In $f [y/n]? " && sed -i '' "s,$hardcodedLibPythonPath,$localLibPythonPath,g" $f
    done
}

fixHardCodedInstallDir() {
    local hardcodedInstallDir=/Users/runner/work/ROS2-On-iOS/ROS2-On-iOS/ros2_macOS
    local hardcodedFiles=($(grep -r -l --include="*.cmake" --include="*.pc" --include="*.zsh" --include="*.sh" --include="*.bash" "$hardcodedInstallDir" .))

    echo "Replace hardcoded path $hardcodedInstallDir -> $installDir"

    for f in "${hardcodedFiles[@]}"; do
        confirm "In $f [y/n]? " && sed -i '' "s,$hardcodedInstallDir,$installDir,g" $f
    done
}

fixXcodeSDKPath() {
    local hardcodedMacOSSdkDir=/Applications/Xcode_13.2.1.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX12.1.sdk/
    local macOSSdkDir=$(xcodebuild -version -sdk macosx Path) # /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/
    local hardcodedFiles=($(grep -r -l --include="*.cmake" --include="*.pc" "$hardcodedMacOSSdkDir" .))

    echo "Replace hardcoded path $hardcodedMacOSSdkDir -> $macOSSdkDir"

    for f in "${hardcodedFiles[@]}"; do
        confirm "In $f [y/n]? " && sed -i '' "s,$hardcodedMacOSSdkDir,$macOSSdkDir,g" $f
    done
}

cd $installDir

echo ""
echo "Fix hardcoded Python environment path ..."
echo ""
fixHardCodedPythonEnvPath

echo ""
echo "Fix hardcoded path to libpython*.dylib ..."
echo ""
fixHardCodedLibPythonPath

echo ""
echo "Fix hardcoded ROS2 and dependencies installation location ..."
echo ""
fixHardCodedInstallDir

echo ""
echo "Fix hardcoded MacOS SDK path ..."
echo ""
fixXcodeSDKPath

echo ""
echo "REMINDERS"
echo "You also need to change the path to OpenGL.framework in $installDir/rviz2/share/rviz_default_plugins/cmake/rviz_default_pluginsExport.cmake to match your Xcode's installation and the python3 path in $installDir/base/bin/ros2."
echo ""
