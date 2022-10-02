localPythonEnvPath=$1 # e.g. $HOME/usr/ros2PythonEnv
localLibPythonPath=$2 # e.g. /Library/Frameworks/Python.framework/Versions/3.10/lib/python3.10/config-3.10-darwin/libpython3.10.dylib

fixHardCodedPythonEnvPath() {
    hardcodedPythonEnvPath=/Users/runner/work/ROS2-On-iOS/ROS2-On-iOS/ros2PythonEnv

    echo "Replacing $hardcodedPythonEnvPath with $localPythonEnvPath"

    # List down files which hard-coded the python environment paths on GitHub Action
    hardcodedFiles=$(grep -r -l "$hardcodedPythonEnvPath" .)

    # Replace the hardcoded path in those files
    for f in $hardcodedFiles; do
        echo "In file $f"
        sed -i.bak "s,$hardcodedPythonEnvPath,$localPythonEnvPath,g" $f
    done
}

fixHardCodedLibPythonPath() {
    local hardcodedLibPythonPath=/usr/local/opt/python@3.10/Frameworks/Python.framework/Versions/3.10/lib/python3.10/config-3.10-darwin/libpython3.10.dylib

    local hardcodedFiles=$(grep -r -l "$hardcodedLibPythonPath" .)

    echo $hardcodedFiles

    # Replace the hardcoded path in those files
    for f in $hardcodedFiles; do
        echo "In file $f"
        sed -i.bak "s,$hardcodedLibPythonPath,$localLibPythonPath,g" $f
    done
}

echo ""
echo "Fix hardcoded Python environment path ..."
echo ""
fixHardCodedPythonEnvPath

echo ""
echo "Fix hardcoded path to libpython*.dylib ..."
echo ""
fixHardCodedLibPythonPath

# Delete the backup files generated
echo ""
echo "Delete backup files generated (make sure they are safe to delete, skip if not sure) ..."
echo ""
find . -name \*.bak -ok rm {} \;
