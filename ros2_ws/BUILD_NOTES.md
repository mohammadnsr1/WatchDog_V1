# Build Notes

This ROS 2 Jazzy workspace should build with the system Python at `/usr/bin/python3`.

Without forcing that interpreter, CMake may instead detect `/usr/local/bin/python3.11`, which can break ROSIDL interface generation because that Python environment does not provide the required ROS 2 and EmPy setup for this workspace.

To keep builds reproducible at the project level, this workspace includes `colcon_defaults.yaml` with `Python3_EXECUTABLE` pinned to `/usr/bin/python3`.

Build manually from the workspace root with:

```bash
colcon build
```
