# Bee Buttons ROS 2 interface

## Installing

### Using `rosdep`

Run the following command to install all dependencies declared in `package.xml` packages inside this workspace:

```bash
rosdep install --from-paths src --ignore-src  # add the `-y` flag to automatically install all dependencies
```

### Virtual environment (WIP)

**Note: these instructions do not work properly yet! More testing is required to get installation in a virtual environment working.**

If you do not want to pollute the global installation of Python (which is recommended), the following steps allow you to install the additional dependencies to a virtual environment and run ROS from that environment.

**Important:** If you use `rosdep` to install dependencies which include Python packages, they will probably be installed globally. Use that command with caution.

0. Install dependencies:

   ```bash
   sudo apt install python3-venv
   ```

1. In your workspace folder, source ROS to obtain links to the ROS system packages and create a virtual environment with these packages

   ```bash
   cd ~/your_workspace
   source /opt/ros/humble/setup.bash
   python3 -m venv .venv --system-site-packages --symlinks
   ```

2. Add a `COLCON_IGNORE` file to the virtual environment folder to prevent `colcon` from trying to build the `.venv` contents

   ```bash
   touch .venv/COLCON_IGNORE
   ```

3. Add the following snippet to `setup.cfg` of all packages that will use dependencies installed in the virtual environment:

   ```text
   [build_scripts]
   executable = /usr/bin/env python3
   ```

4. Activate the virtual environment and install python packages (you can repeat this step whenever you want to install additional packages):

   ```bash
   source .venv/bin/activate

   # Directly
   python3 -m pip install package1 package2

   # OR from a requirements.txt file
   # TODO: Perhaps there is a way to get all requirements.txt files from files inside the src/ folder?
   python3 -m pip install -r path/to/package/requirements.txt
   ```

5. Build the workspace with the virtual environment sourced (you can repeat this step whenever you want to build the workspace):

   ```bash
   source .venv/bin/activate
   colcon build
   ```

6. Source the built files and run the nodes. Make sure that the virtual environment is still sources (you can repeat this step whenever you want to run or launch):

   ```bash
   source .venv/bin/activate
   source install/setup.bash
   ros2 run package_name node_name
   ```

These steps are based on the following resources:

- <https://medium.com/ros2-tips-and-tricks/running-ros2-nodes-in-a-python-virtual-environment-b31c1b863cdb>
- <https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html>
- <https://www.theconstruct.ai/ros2-how-to-install-third-party-python-packages-using-ros2-5/>

## Usage

### Set up permissions

Before starting the node, you need to assign the proper rights to access the serial device. If you do not do this, you will likely get an error such as:

```text
[Errno 13] could not open port /dev/ttyACM0: [Errno 13] Permission denied: '/dev/ttyACM0'
```

You have two options:

- Set up access rights for this session (resets when the logging out or PC shuts down):

  ```bash
  sudo chmod a+rw /dev/ttyACM0
  ```

- Add the current user to the `dialout` group to gain access permanently:

  ```bash
  sudo usermod -a -G dialout $USER
  ```

  Make sure to log out and in again to make the changes to into effect.
