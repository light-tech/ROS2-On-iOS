platform=$1
resourceUrl=https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-1.4

fetchCachedPythonEnvAndPrebuiltDeps() {
    curl -OL $resourceUrl/python_venv.tar.xz \
         -O  $resourceUrl/deps_$platform.tar.xz
}

fetchPrebuiltWorkspace() {
    workspaceToBuild=$1
    case $workspaceToBuild in
        "base")
            echo "No dependent workspace for ROS2 base";;

        "rviz2")
            curl -OL $resourceUrl/base_$platform.tar.xz;;

        "moveit2")
            curl -OL $resourceUrl/base_$platform.tar.xz -O $resourceUrl/rviz2_$platform.tar.xz;;

        "tutorials")
            curl -OL $resourceUrl/base_$platform.tar.xz -O $resourceUrl/rviz2_$platform.tar.xz -O $resourceUrl/moveit2_$platform.tar.xz;;

        *)
            echo "Unknown workspace to build";;
    esac
}

extractArtifacts() {
    find . -name "*.tar.xz" -exec tar xzf {} \;
}
