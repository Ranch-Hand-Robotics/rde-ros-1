// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";
import { rosApi } from "../../../ros/ros";

// interact with the user to create a roslaunch or rosrun configuration
export class RosDebugConfigurationProvider implements vscode.DebugConfigurationProvider {
    public async provideDebugConfigurations(
        folder: vscode.WorkspaceFolder | undefined,
        token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration[]> {

        if (token?.isCancellationRequested) {
            return [];
        }

        // When VS Code asks for initial configurations (user selected "ROS 2" from dropdown),
        // trigger the interactive flow to find actual packages and launch files
        return this.provideDebugConfigurationsInteractive(folder, token);
    }

    /**
     * Interactive configuration method for when user needs more detailed setup
     */
    public async provideDebugConfigurationsInteractive(
        folder: vscode.WorkspaceFolder | undefined,
        token?: vscode.CancellationToken): Promise<vscode.DebugConfiguration[]> {              const type = await vscode.window.showQuickPick(
            ["ROS 1: Launch", "ROS 1: Attach"], { placeHolder: "Choose a request type" });
        
            if (!type) {
            return [];
        }

        switch (type) {
            case "ROS 1: Launch": {
                const packageName = await vscode.window.showQuickPick(rosApi.getPackageNames(), {
                    placeHolder: "Choose a package",
                });
                if (!packageName) {
                    return [];
                }
                const launchFiles = (await rosApi.findPackageLaunchFiles(packageName)).concat(await rosApi.findPackageTestFiles(packageName));
                const launchFileBasenames = launchFiles.map((filename) => path.basename(filename));
                const target = await vscode.window.showQuickPick(
                    launchFileBasenames, { placeHolder: "Choose a launch file" });
                const launchFilePath = launchFiles[launchFileBasenames.indexOf(target)];
                if (!launchFilePath) {
                    return [];
                }

                return [{
                    name: type,
                    request: "launch",
                    target: `${launchFilePath}`,
                    launch: ["rviz", "gz", "gzclient", "gzserver"],
                    type: "ros",
                }];            }
            case "ROS 1: Attach": {
                return [{
                    name: "ROS 1: Attach",
                    request: "attach",
                    type: "ros",
                }];
            }
        }

        return [];
    }
}
