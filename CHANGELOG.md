# Changelog

All notable changes to the Robot Developer Extensions for ROS 1 will be documented in this file.

## [1.0.5] - 2025-12-27

### Added
* Support for additional ROS 1 launch file debugging scenarios
* Enhanced error messages for workspace configuration issues
* Improved process picker functionality on multiple platforms

### Fixed
* Fixed debugger attachment timeouts on slow systems
* Corrected Python path detection for virtual environments
* Resolved issues with launch file argument parsing

### Changed
* Improved performance of workspace scanning for ROS packages
* Enhanced terminal output formatting for ROS commands

## [1.0.4] - 2025-11-15

### Added
* Advanced debugging capabilities with process selection UI
* Support for attaching debuggers to running ROS nodes
* Integration with VS Code's native debuggers (C++, Python)

### Fixed
* Fixed source map generation for debugger symbols
* Corrected environment variable propagation in debug sessions

## [1.0.3] - 2025-10-01

### Added
* ROS 1 core status monitoring webview
* URDF file preview and validation

### Fixed
* Performance optimizations for large catkin workspaces
* Improved caching of ROS package metadata

## [1.0.2] - 2025-08-20

### Added
* Additional ROS 1 build tool integrations
* Support for catkin_make and catkin_make_isolated
* Custom problem matchers for build output parsing

### Fixed
* Fixed C++ property updates for IntelliSense
* Corrected Python path updates for linting

## [1.0.1] - 2025-06-10

### Added
* Core ROS 1 extension functionality
* Support for rosrun and roslaunch commands
* ROS message/service/action syntax highlighting and language support
* Catkin package creation utilities

### Fixed
* Fixed initialization of ROS environment detection
* Improved compatibility with various ROS distributions

## [0.0.1] - 2025-04-15

### Added
* Initial release of Robot Developer Environment for ROS 1
* Basic ROS workspace detection
* Launch file syntax support
