# Groot2 for PB2025 UL behavior trees

Groot2 is installed locally inside this workspace. It does not write to `/opt` or system package folders.

Open the UL behavior-tree project on the robot desktop:

```bash
cd ~/version2/pb2025_sp2025_sentry_ws/pb2025_sp2025_sentry_ws/src/pb2025_sentry_behavior/tools/groot2
./open_groot2.sh
```

The project file is:

```text
~/version2/pb2025_sp2025_sentry_ws/pb2025_sp2025_sentry_ws/src/pb2025_sentry_behavior/behavior_trees/Project.btproj
```

If you run this from a pure SSH terminal without X11 forwarding or robot desktop display access, Qt may report `could not connect to display`. Run it on the robot desktop terminal, or SSH with X11 forwarding.

To monitor a running behavior tree in real time: click **Connect** in Groot2, set Host to `localhost` and Port to `1667`.