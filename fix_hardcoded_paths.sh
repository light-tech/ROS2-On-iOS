installDir=$1           # path to the ROS2 installation location e.g. $HOME/usr/ros2
localPythonEnvPath=$2   # path to virtual Python environment on local machine e.g. $HOME/usr/ros2PythonEnv
localLibPythonPath=$3   # path to libpython dylib e.g. /Library/Frameworks/Python.framework/Versions/3.10/lib/python3.10/config-3.10-darwin/libpython3.10.dylib

fixHardCodedPythonEnvPath() {
    hardcodedPythonEnvPath=/Users/runner/work/ROS2-On-iOS/ROS2-On-iOS/ros2PythonEnv
    hardcodedFiles=($(grep -r -l --include="*.cmake" "$hardcodedPythonEnvPath" .))

    for f in $hardcodedFiles; do
        echo "Replacing $hardcodedPythonEnvPath -> $localPythonEnvPath in file $f"
        sed -i '' "s,$hardcodedPythonEnvPath,$localPythonEnvPath,g" $f
    done
}

fixHardCodedLibPythonPath() {
    local hardcodedLibPythonPath=/usr/local/opt/python@3.10/Frameworks/Python.framework/Versions/3.10/lib/python3.10/config-3.10-darwin/libpython3.10.dylib
    local hardcodedFiles=($(grep -r -l --include="*.cmake" "$hardcodedLibPythonPath" .))

    for f in $hardcodedFiles; do
        echo "Replacing $hardcodedLibPythonPath -> $localLibPythonPath in file $f"
        sed -i '' "s,$hardcodedLibPythonPath,$localLibPythonPath,g" $f
    done
}

fixHardCodedInstallDir() {
    local hardcodedInstallDir=/Users/runner/work/ROS2-On-iOS/ROS2-On-iOS/ros2_macOS
    local hardcodedFiles=($(grep -r -l --include="*.cmake" --include="*.pc" "$hardcodedInstallDir" .))

    for f in $hardcodedFiles; do
        echo "Replace hardcoded path $hardcodedInstallDir -> $installDir in file $f"
        sed -i '' "s,$hardcodedInstallDir,$installDir,g" $f
    done
}

fixXcodeFrameworkPath() {
    echo "You also need to change the OpenGL.framework path in $installDir/share/rviz_default_plugins/cmake/rviz_default_pluginsExport.cmake to match your Xcode's installation."
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

fixXcodeFrameworkPath
