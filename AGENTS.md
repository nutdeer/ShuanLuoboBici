# Repository Guidelines

## Project Structure & Module Organization

This directory is the `src` tree of a ROS 1 catkin workspace. `epic/global_planner/` contains exploration, frontier, LiDAR map, and point-cloud topology packages; `epic/local_planner/` contains trajectory planning and path search. `MARSIM/` provides simulation packages, `Utils/` holds shared utilities and ROS messages, and `px4ctrl/` contains flight-control code. `FAST_LIO/` contains LiDAR-inertial odometry but is currently excluded from catkin discovery by `CATKIN_IGNORE`. Packages typically keep implementation in `src/`, headers in `include/`, and launch files, parameters, and visualization assets in `launch/`, `config/`, and `rviz/`.

## Build, Test, and Development Commands

Run these from the workspace root, one directory above this repository:

- `source /opt/ros/noetic/setup.bash` prepares the ROS environment.
- `catkin_make` builds discovered packages; install their ROS and system dependencies first.
- `source devel/setup.bash` exposes the built packages to ROS tools.
- `catkin_make run_tests` runs registered catkin tests; `catkin_test_results` reports results.
- `roslaunch epic_planner garage.launch` starts the documented exploration entry point. Select an appropriate launch file and configuration for your environment.

## Coding Style & Naming Conventions

Follow the surrounding package's C++ formatting and indentation; styles vary across packages. Use descriptive `snake_case` for new files and ROS parameters, and keep public headers under `include/<package>/`. Respect each package's C++ standard: for example, `pointcloud_topo` uses C++17. Keep ROS dependencies and build targets declared in the package's `package.xml` and `CMakeLists.txt`.

## Testing Guidelines

Add focused tests for behavioral changes and register them with catkin where practical. `Utils/uav_utils` uses `catkin_add_gtest`; follow its pattern for Google Test. Files named `test_*.cpp` exist in `epic/global_planner/pointcloud_topo/test/`, but its CMake file does not currently register them. For planning or simulation changes, also exercise a relevant launch scenario and record the environment and observed behavior. No repository-wide coverage threshold is specified.

## Commit & Pull Request Guidelines

Recent commit subjects are short, descriptive summaries, often in Chinese. Use a concise subject that describes the change; add context in the body when needed. Pull requests should explain the problem and approach, link related issues when available, and report build, test, and simulation results. Include screenshots or RViz recordings for visual changes and identify configuration or hardware dependencies.
