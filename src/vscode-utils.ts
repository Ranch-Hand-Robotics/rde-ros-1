// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

export interface IPackageInfo {
    name: string;
    version: string;
    aiKey: string;
}

export function getPackageInfo(extensionId: string): IPackageInfo {
    const extension = vscode.extensions.getExtension(extensionId);
    const metadata = extension.packageJSON;
    if (metadata && ("name" in metadata) && ("version" in metadata) && ("aiKey" in metadata)) {
        return {
            name: metadata.name,
            version: metadata.version,
            aiKey: metadata.aiKey,
        };
    }
    return undefined;
}

export function getExtensionConfiguration(): vscode.WorkspaceConfiguration {
    const rosConfigurationName: string = "ros";
    return vscode.workspace.getConfiguration(rosConfigurationName);
}

export function createOutputChannel(): vscode.OutputChannel {
    return vscode.window.createOutputChannel("ROS");
}


/**
 * Detects if the extension is running in Cursor (Anysphere's editor)
 * @returns true if running in Cursor, false otherwise
 */
export function isRunningInCursor(): boolean {
    // Method 1: Check for Cursor-specific environment variables
    if (process.env.CURSOR_EXTENSION_HOST || process.env.ANYSPHERE_EXTENSION_HOST) {
        return true;
    }

    // Method 2: Check for Cursor-specific process name patterns
    if (process.env.VSCODE_PID) {
        try {
            // On Linux, we can check the process name
            if (process.platform === 'linux') {
                const fs = require('fs');
                const procPath = `/proc/${process.env.VSCODE_PID}/comm`;
                if (fs.existsSync(procPath)) {
                    const processName = fs.readFileSync(procPath, 'utf8').trim();
                    if (processName.includes('cursor') || processName.includes('Cursor')) {
                        return true;
                    }
                }
            }
        } catch (error) {
            // Ignore errors in process detection
        }
    }

    // Method 3: Check for Cursor-specific extension dependencies
    // This method was removed due to webpack bundling issues with require('../../package.json')
    // The other detection methods should be sufficient for most cases

    // Method 4: Check for Cursor-specific workspace settings or configurations
    const workspaceConfig = vscode.workspace.getConfiguration();
    const cursorSpecificSettings = [
        'cursor',
        'anysphere',
        'cursorExtensionHost'
    ];

    for (const setting of cursorSpecificSettings) {
        if (workspaceConfig.has(setting)) {
            return true;
        }
    }

    // Method 5: Check for Cursor-specific command line arguments
    if (process.argv.some(arg => arg.toLowerCase().includes('cursor'))) {
        return true;
    }

    return false;
}