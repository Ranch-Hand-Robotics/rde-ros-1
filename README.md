# Robot Developer Extensions for ROS 1
This is a Visual Studio Code Extension that provides debugging support for [Robot Operating System (ROS 1)](http://ros.org) development. The Robot Operating System is a trademark of Open Robotics.

> NOTE: This extension is rebranded and re-released by Ranch Hand Robotics, a company owned by the maintainer of the [ms-iot VSCode ROS Extension](https://github.com/ms-iot/vscode-ros) with permission from Microsoft. The source extension was split into 3 parts - [ROS 1](https://ranchhandrobotics.github.io/rde-ros-1/), [ROS 2](https://ranchhandrobotics.github.io/rde-ros-2/) and a [URDF/Xacro editor](https://ranchhandrobotics.github.io/rde-urdf/).


## Features

* Automatic ROS environment configuration.
* Allows starting, stopping and viewing the ROS core status.
* Automatically create `catkin_make` or `catkin build` build tasks.
* Create catkin packages using `catkin_create_pkg` script or `catkin create pkg`.
* Run `rosrun` and `roslaunch`
* Resolve dependencies with `rosdep` shortcut
* Syntax highlighting for `.msg`, `.urdf` and other ROS files.
* Automatically add the ROS C++ include and Python import paths.
* Format C++ using the ROS `clang-format` style.
* Debug a single ROS node (C++ or Python) by [attaching to the process](https://ranchhandrobotics.com/rde-ros-1/debug-support.html#attach).
* Debug ROS nodes (C++ or Python) [launched from a `.launch` file](https://ranchhandrobotics.com/rde-ros-1/debug-support.html#launch).
* Support for Visual Studio Code and Cursor

## Getting Started

The VS Code ROS extension will attempt to detect and automatically configure the workspace for the appropriate ROS Distro.

The extension will automatically start when you open a `catkin` workspace.
The build system (e.g. catkin_make or catkin build) will automatically be confirmed from the hidden files associated with
each system. 

## Available Commands
Visit the [Tutorials and Walkthroughs](https://github.com/ranch-hand-robotics/rde-ros-1/blob/master/docs/usage.md)

## Tutorials and Walkthroughs

Visit the [Tutorials and Walkthroughs](https://github.com/ranch-hand-robotics/rde-ros-1/blob/master/docs/tutorials.md)


## Configuration
  
The configuration options for the ROS extension enable customization and setup according to your specific needs. Refer to the [ROS Launch Configuration options](https://github.com/ranch-hand-robotics/rde-ros-1/blob/master/docs/configuration.md) for detailed descriptions of available settings.

## Launch Debugging
  
The Launch Debugging feature allows users to debug ROS nodes that are initiated by a launch file. 

For more detailed instructions and options, refer to our [debugging documentation](https://github.com/ranch-hand-robotics/rde-ros-1/blob/master/docs/launchdebugging.md).


## Support
If you encounter any issues with this extension, the following resources are provided:

### Github Issues
Bugs and feature requests are handled through [Github Issues in the Repository](https://github.com/Ranch-Hand-Robotics/rde-ros-1/issues). 
If you find that you are hitting the same issue as someone else, please give a :+1: or comment on an existing issue.
Please provide as much details as possible, including an isolated reproduction of the issue or a pointer to an online repository.

### Discussions
[Github Discussions](https://github.com/orgs/Ranch-Hand-Robotics/discussions) are provided for community driven general guidance, walkthroughs, or support.

## Sponsor
If you find this extension useful, please consider [sponsoring Ranch Hand Robotics](https://github.com/sponsors/Ranch-Hand-Robotics) to help support the development of this extension and other open source projects.


## Contribution
Contributions are always welcome! Please see our [contributing guide][contributing] for more details!

A big ***Thank you!*** to everyone that have helped make this extension better!

* Andrew Short ([@ajshort](https://github.com/ajshort)), **original author**
* James Giller ([@JamesGiller](https://github.com/JamesGiller))
* PickNikRobotics ([@PickNikRobotics](https://github.com/PickNikRobotics)) for code formatting

<!-- link to files -->
<!-- relative links in Visual Studio Marketplace page lead to 404 error, need to use absolute link -->
[contributing]: https://github.com/ranch-hand-robotics/rde-ros-1/blob/master/CONTRIBUTING.md

<!-- feature documentation -->
[debug_support-attach]: https://github.com/ranch-hand-robotics/rde-ros-1/blob/master/docs/debug-support.md#attach
[debug_support-launch]: https://github.com/ranch-hand-robotics/rde-ros-1/blob/master/docs/debug-support.md#launch

<!-- media -->
[download_vsix_artifact]: https://raw.githubusercontent.com/ranch-hand-robotics/rde-ros-1/master/docs/assets/download-vsix-artifact.png

