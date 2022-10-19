# Build ROS2 graphical tools for macOS

While there is an option, namely [RoboStack](https://robostack.github.io/GettingStarted.html), to install ROS2 for macOS without having to build it from source, their prebuilt graphical tools such as `rviz2` and `gazebo` do not work.

So let us build ROS2 from source to use on macOS (Intel only) as well.

## For the impatient

If you want to save yourself one full working day, you can use [our release](https://github.com/light-tech/ROS2-On-iOS/releases) built using GitHub Action.

 *  If you download the file from a browser, it will be put in quarantine so you need to `xattr -d com.apple.quarantine DOWNLOADED_FILE` before extraction.
 *  To avoid that thank to [this discussion thread](https://developer.apple.com/forums/thread/703523), you could open the terminal and do
    ```shell
    curl -OL https://github.com/light-tech/ROS2-On-iOS/releases/download/humble-1.2/ros2_macOS.tar.xz
    ```

**Note**: We did not manage to build native ROS2 for ARM64 Mac but you can still use the Intel version thank to [Rosetta 2](https://support.apple.com/en-us/HT211861).
In that case, you will need the `x86_64` version of Python.

After extracting the archive with `tar xzf` and move it to where you want (I usually rename the folder to `ros2` and move it inside `~/usr` in my home folder along with the other Linux-based software), you first need to
```shell
# Tip: Add these commands to your `.zshrc` to have them ready.

export DYLD_LIBRARY_PATH=PATH_TO_EXTRACTED_ROS2/deps/lib:$DYLD_LIBRARY_PATH
export DYLD_FRAMEWORK_PATH=/Library/Frameworks:$DYLD_FRAMEWORK_PATH # So that ros2 can find the Python framework that you installed yourself

source PATH_TO_PYTHON_ENV/bin/activate
source PATH_TO_EXTRACTED_ROS2/setup.zsh
```

Next, you need to fix hardcoded paths to match YOUR system as `colcon` (or maybe CMake) unfortunately hardcoded a lot of paths (mostly concerning Python-based stuffs) in generated files such as CMake and package config files.
To do that, run our script `fix_hardcoded_paths.sh` and you will be prompt with the relevant information.
The script is not exhaustive so you will need to manually fix the path such as the path to `python3` in `bin/ros2` so you can run `ros2` command line such as `ros2 launch` and `ros2 topic list`.

### Note about building and using overlaid workspace

 1. You might want to add `-DCMAKE_PREFIX_PATH` such as
    ```shell
    colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=$ros2SystemDependenciesPath
    ```
    to expose our system dependencies.
    Here, `$ros2SystemDependenciesPath` should be `PATH_TO_EXTRACTED_ROS2/deps`.

 2. Because of [this issue](https://github.com/colcon/colcon-zsh/issues/12) where the generated `setup.zsh` might NOT set the library search path for system libraries (Qt, FreeType2, Bullet ...), tools that depend on our locally built `dylib` in `$ros2SystemDependenciesPath` will fail to load.
    (The official instruction works because `homebrew` will put the libraries in globally available location `/usr/local`.)
    So to actually use your workspace you might need to `source` it
    ```shell
    source YOUR_WORKSPACE/install/setup.zsh
    ```
    and then
    ```shell
    export DYLD_LIBRARY_PATH=$ros2SystemDependenciesPath/lib:$DYLD_LIBRARY_PATH
    ```
    You must do this in this order because `setup.zsh` might not extend but *overwrite* `DYLD_LIBRARY_PATH`.

### Getting started

Now you can use ROS2 [as usual](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html).

  * Try following [this tutorial](https://github.com/ros-perception/image_transport_tutorials). You need to grab <del>the `humble` branch of [our small addition to vision_opencv](https://github.com/light-tech/vision_opencv)</del> [`vision_opencv`](https://github.com/ros-perception/vision_opencv) in your workspace <del>as it adds `CV_BRIDGE_ENABLE_PYTHON` to disable Python stuffs in `cv_bridge` package (requires Boost Python that we did not build)</del> and then
    ```shell
    colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=$PATH_TO_EXTRACTED_ROS2/deps -DBUILD_TESTING=NO

    # To run the examples

    ros2 run image_transport_tutorials my_publisher path/to/some/image.jpg
    ros2 run image_transport_tutorials my_subscriber
    ```

  * The [URDF tutorial](https://github.com/ros/urdf_tutorial) also works flawlessly.

## Building

Please see our script [`build_ros2.sh`](build_ros2.sh) for details.
