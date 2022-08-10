# ROS2 on iOS

Build ROS2 stack for iOS software development.

Guides:

 * https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html
 * https://docs.ros.org/en/humble/How-To-Guides/Cross-compilation.html

Steps:

 1. Create a new Python virtual environment
    ```shell
    export MY_PYTHON_ENV=usr_local
    python3 -m venv $MY_PYTHON_ENV
    ```
    Here I use `usr_local` as it is intended to act like `/usr/local/`.

 2. Activate the environment
    ```shell
    source $MY_PYTHON_ENV/bin/activate
    ```

 3. Install the packages
    ```shell
    python3 -m pip install -r requirements.txt
    ```
