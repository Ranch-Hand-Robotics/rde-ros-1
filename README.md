# Robot Developer Extensions for ROS 1

The Robot Developer Extensions (RDE) is a set of extensions for [Visual Studio Code][vscode]. This extension provides support for the Legacy [Robot Operating System (ROS)][ros] 1 runtime, an open-source robotics middleware producted by [Open Robotics](https://www.openrobotics.org/) and supports development on Windows and Linux. 

> NOTE: This extension is rebranded and re-released by Ranch Hand Robotics, owned by the maintainer of the [MS-IOT VSCode ROS Extension](https://github.com/ms-iot/vscode-ros) with permission from Microsoft. 

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
* Preview URDF and Xacro files.
* Debug a single ROS node (C++ or Python) by [attaching to the process][debug_support-attach].
* Debug ROS nodes (C++ or Python) [launched from a `.launch` file][debug_support-launch].

## Commands

You can access the following commands from the [Visual Studio Code command pallet](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette), typically accessed by pressing `ctrl` + `shift` + `p` and typing the command name you'd like to use from the table below.

| Name | Description |
|---|:---|
| ROS: Create Catkin Package | Create a catkin package. You can right click on a folder in the explorer to create it in a specific location. |
| ROS: Create Terminal | Create a terminal with the ROS environment. |
| ROS: Show Status | Open a detail view showing ROS core runtime status. |
| ROS: Start | Start ROS core. |
| ROS: Stop  | Terminate ROS core. |
| ROS: Update C++ Properties | Update the C++ IntelliSense configuration to include ROS and your ROS components. |
| ROS: Update Python Path | Update the Python IntelliSense configuration to include ROS. |
| ROS: Preview URDF | Preview URDF and Xacro files. The display will update after the root URDF changes are saved. |
| ROS: Install ROS Dependencies for this workspace using rosdep | Shortcut for `rosdep install --from-paths src --ignore-src -r -y`. |

## Tutorials and Walkthroughs

| Name | Description |
|---|:---|
| [Attaching to a running ROS Node](https://github.com/ranchhandrobotics/rde-ros-1/blob/master/docs/debug-support.md) | Learn how to attach VS Code to a running ROS node |
| [Debugging all ROS Nodes in a launch file ](https://github.com/ranchhandrobotics/rde-ros-1/blob/master/docs/debug-support.md) | Learn how to set up VS Code to debug the nodes in a ROS Launch file |
| [ROSCON 2019 ROS Extension Talk Video](https://vimeopro.com/osrfoundation/roscon-2019/video/379127667) | Walkthrough of VS Code from ROSCon 2019|
| [Deep Dive - Episode 0](https://youtu.be/PBbEhRf8QjE) |  About the VS Code ROS extension @ a Polyhobbyist |
| [Deep Dive - Episode 1](https://youtu.be/bupAju0UAMg) |  Installing on Windows & WSL @ a Polyhobbyist |
| [Deep Dive - Episode 2](https://youtu.be/-NaEPoIg2Ds) |  Installing on Linux @ a Polyhobbyist |
| [Deep Dive - Episode 3](https://youtu.be/N2vqBvPQdhE) |  General Usage with ROS1 @ a Polyhobbyist |
| [Deep Dive - Episode 4](https://youtu.be/k2TLdXHjVsU) |  General Usage with ROS2 @ a Polyhobbyist |
| [Deep Dive - Episode 5](https://youtu.be/A6ABRdL0ckg) |  Debugging Python @ a Polyhobbyist |
| [Deep Dive - Episode 6](https://youtu.be/uqqHgYsskJI) |  Debugging C++ @ a Polyhobbyist |
| [Deep Dive - Episode 7](https://youtu.be/teA20AjBlG8) |  Using with SSH @ a Polyhobbyist |
| [Deep Dive - Episode 8](https://youtu.be/JbBMF1aot5k) |  Using with with Containers @ a Polyhobbyist |

## Getting Started

The VS Code ROS extension will attempt to detect and automatically configure the workspace for the appropriate ROS Distro.

The extension will automatically start when you open a `catkin` workspace.
The build system (e.g. catkin_make or catkin build) will automatically be confirmed from the hidden files associated with
each system. 


## Launch Debugging

The Visual Studio Code extension for ROS supports launch debugging for ROS nodes written in Python and C++. The ROS node or nodes to be debugged must be placed in a ROS launch file with the extension `.launch`. 

### Automatic creation of a launch.json with ROS Launch support
`.vscode/launch.json` is a file which defines a debug launch configuration within VS Code. 

To create a `.vscode/launch.json` with ROS debugging support 

  1. C++ or Python file is selected, vscode uses the selected file to seed the launch creation UI. 
  1. Click the `Run and Debug` tab on the left sidebar
  1. Select the link to create a `.vscode/launch.json` file. 
  1. VS Code will drop down from the command pallet with a list of options, which includes 'ROS'. Select this option. 
  1. In the next dialog, type the name of the ROS package containing a launch file you'd like to debug. 
  1. Then find the launch file.

Once this is created, you can use the play button in the title bar, or the "start debugging" accelerator key, or from the command palle (CTRL-SHIFT-P), select `Debug: Start Debugging`.

> NOTE: Other VS Code extensions may interfere with the selection list. If you do not see ROS in the first drop down list, you'll need to create a new file called `.vscode/launch.json`, then use the manual option described below.

### Manually adding a launch file to an existing launch.json
If you have an existing `launch.json` file (or if there is an extension conflict as mentioned above), you can manually add a launch configuration by adding a new block like this. 
```json
{
    "version": "0.2.0",
    "configurations": [
      {
          "name": "ROS: Launch my file",
          "request": "launch",
          "target": "<full path to your launch.py or launch file>",
          "launch": ["rviz", "gz", "gzserver", "gzclient"],
          "type": "ros"
      }
    ]
}  
```
Be sure to include the full path to your launch file, including file extension.

### ROS Launch Configuration options
The ROS Launch configuration block supports the following configuration:

| Option | Description |
|---|:---|
| name | The name which will be displayed in the VS Code UI launch configuration |
| request | `launch` or `attach` for launching a ROS launch file, or attaching using the attach UI for Pyton or C++ |
| target | the launch file path |
| type | must be `ros` to indicate to VS Code that this is a ROS launch configuration |
| arguments | Arguments passed to roslaunch such as `map:=/foo.yaml'`|
| symbolSearchPath | A semicolon delimited search path for Windows symbols, including ROS for Windows symbols downloaded from https://ros-win.visualstudio.com/ros-win/_build |
| additionalSOLibSearchPath | A semicolon delimited search path for Linux symbols |
| sourceFileMap | A mapping of Source files from where Symbols expect and the location you have on disk. |
| launch | If specified, a list of executables to just launch, attaching to everything else. e.g. `"launch": ["rviz", "gz", "gzserver", "gzclient"]` which prevents attaching a debugger to rviz and gazebo. NOTE: the debugger will ignore file extension: x.py is the same as x.exe. |
| attachDebugger | If specified, a list of executables to debug. `"attachDebugger": ["my_ros_node"]` will only attach to my_ros_node.exe, my_ros_node.py or my_ros_node. |

### Workspace and Global Settings
The ROS extension supports the following global settings, which can be overridden in the workspace.

| Json Option | Setting Name | Description |
|---|:---|---|
| ros.distro | ROS installation distro to be sourced | The Distribution to be sourced. On linux, this cause the extension to look for the ROS setup script in `/opt/ros/{distro}/setup.bash`. On Windows, `c:\opt\ros\{distro}\setup.bat` | 
| ros.rosSetupScript | ROS workspace setup script. Overrides ros.distro. | If specified, this will cause the extension to source this script before generating the launch debugging or ROS terminal environment. This overrides the ros.distro, and can be used to specify user scripts or ROS installs in a different location. |
| ros.isolateEnvironment | Specify if the extension should not capture the environment VS Code is running in to pass to child processes. | Off by default, This setting will prevent the ROS extension from capturing it's hosting environment in case this would conflict with the ROS environment. |

Workspace example:

``` bash
└── .vscode
    ├── launch.json
    ├── settings.json
    └── tasks.json
```

`settings.json`
```json
{
    "ros.distro": "foxy",
    "ros.rosSetupScript": "/opt/ros/noetic/install/setup.bash",
    "ros.isolateEnvironment": "false"
}
```


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
[download_vsix_artifact]: https://raw.githubusercontent.com/ranch-hand-robotics/rde-ros-1/master/media/documentation/download-vsix-artifact.png

<!-- link to external sites -->
[ros]: http://ros.org
[vscode]: https://code.visualstudio.com

