//@ts-check

'use strict';

const path = require('path');

/** @typedef {import('webpack').Configuration} WebpackConfig **/

/** @type WebpackConfig */
const baseConfig = {
  mode: "none", // this leaves the source code as close as possible to the original (when packaging we set this to 'production')
  externals: {
    vscode: "commonjs vscode", // the vscode-module is created on-the-fly and must be excluded. Add other modules that cannot be webpack'ed, 📖 -> https://webpack.js.org/configuration/externals/
    // modules added here also need to be added in the .vscodeignore file
  },
  resolve: {
    extensions: [".ts", ".tsx", ".js"],
  },
  devtool: "source-map",
  infrastructureLogging: {
    level: "log", // enables logging required for problem matchers
  },
  module: {
    rules: [
      {
        test: /\.tsx?$/,
        exclude: /node_modules/,
        use: [{ loader: "ts-loader" }],
      },
    ],
  }
};

// Config for extension source code (to be run in a Node-based context)
/** @type WebpackConfig */
const extensionConfig = {
  ...baseConfig,
  target: "node",
  entry: "./src/extension.ts",
  externals: {
    vscode: 'commonjs vscode', // the vscode-module is created on-the-fly and must be excluded. Add other modules that cannot be webpack'ed, 📖 -> https://webpack.js.org/configuration/externals/
    'applicationinsights-native-metrics': 'commonjs applicationinsights-native-metrics' // ignored because we don't ship native module
  },
  output: {
    path: path.resolve(__dirname, "dist"),
    filename: "extension.js",
    libraryTarget: "commonjs2",
    devtoolModuleFilenameTemplate: '../[resource-path]'
  },
};

/** @type WebpackConfig */
const debuggerConfig = {
  ...baseConfig,
  target: "node",
  entry: "./src/debugger/main.ts",
  externals: {
    vscode: 'commonjs vscode', // the vscode-module is created on-the-fly and must be excluded. Add other modules that cannot be webpack'ed, 📖 -> https://webpack.js.org/configuration/externals/
    '@vscode/debugadapter': 'commonjs @vscode/debugadapter',
    '@vscode/debugprotocol': 'commonjs @vscode/debugprotocol'
  },
  output: {
    path: path.resolve(__dirname, "dist"),
    filename: "debugger-main.js",
    libraryTarget: "commonjs2",
    devtoolModuleFilenameTemplate: '../[resource-path]'
  },
};

/** @type WebpackConfig */
const ros1_webview_config = {
  ...baseConfig,
  target: ["web", "es2022"],
  entry: "./src/ros/ros1/webview/ros1_webview_main.ts",
  experiments: { outputModule: true, topLevelAwait: true },
  resolve: {
    extensions: [".ts", ".tsx", ".js"],
  },
  output: {
    path: path.resolve(__dirname, "dist"),
    filename: "ros1_webview_main.js",
    libraryTarget: "module",
    chunkFormat: "module",
  },
};

module.exports = [extensionConfig, debuggerConfig, ros1_webview_config];
