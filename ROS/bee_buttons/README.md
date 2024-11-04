# Bee Buttons ROS 2 interface

A ROS 2 interface for the Bee Buttons.

## Installation

### Preparing the workspace

This package should be installed inside the `src` folder of a `colcon` workspace, e.g.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this repository URL>
```

### Installing dependencies

Before building, some dependencies are needed.

#### Using `rosdep`

Run the following command to install all dependencies declared in `package.xml` packages inside this workspace:

**Note**: this package depends on pure Python packages, which will be installed into your global Python interpreter. This pollutes your global Python installation, which may not be what you want. Consider using a virtual environment instead (explained below).

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src  # add the `-y` flag to automatically install all dependencies
```

#### Using virtual environment (WIP)

**Warning: these instructions do not work properly yet! More testing is required to get installation in a virtual environment working. Until then, use [the `rosdep` approach instead](#using-rosdep).**

If you do not want to pollute the global installation of Python (which is recommended), the following steps allow you to install the additional dependencies to a virtual environment and run ROS from that environment.

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

### Building the nodes

Run the following commands to build the ROS 2 nodes:

```bash
cd ~/ros2_ws
colcon build
```

### Setup permissions

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

## Usage

All commands in this section assume you are in the workspace folder and have sourced it:

```bash
cd ~/ros2_ws
source install/setup.bash
```

### Run the node

Run the node with the following command:

```bash
ros2 run bee_buttons bee_buttons
```

If you get permission errors, follow the steps in the [Setup permissions section](#setup-permissions)

Press CTRL-C to interrupt the node.

### Configure the node

When the node is running, you can obtain detailed information about the parameters to configure it with:

```bash
ros2 param describe /bee_buttons $(ros2 param list /bee_buttons)
```

You can re-run the node with adjusted parameters with:

```bash
ros2 run bee_buttons bee_buttons --ros-args -p update_rate:=50.0 -p battery_info_on_startup:=false
```

### Read topic contents

While the node is running in one terminal, you can open another terminal and read the ROS topics with:

```bash
ros2 topic echo /button_press bee_buttons_interfaces/msg/BeeButtonPress
```

Or you can read the battery information:

```bash
ros2 topic echo /button_battery_info bee_buttons_interfaces/msg/BeeButtonBatteryInfo
```

## Author

Jelle Hierck (<j.j.hierck@student.utwente.nl>)

## Acknowledgments

The ROS 2 interface was written as part of the thesis of Jelle Hierck at Nakama Robotics Lab (BE-BRT) at the University of Twente.
