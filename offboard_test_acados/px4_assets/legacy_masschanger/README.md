# legacy_masschanger PX4 assets (archived 2026-09-15)

Removed from the shared `/home/clear/PX4-Autopilot` tree after the masschanger
line was retired (`efe7986`). Needed only to re-simulate historical results
listed in `paper/REPRODUCE.md` (A.2 no-signal ablation, C.3 wind FPR); the
paper's current tables are recomputed from logs and do not need these.

| File | Install to |
| --- | --- |
| `models/x500_payload/` | `PX4-Autopilot/Tools/simulation/gz/models/` |
| `airframes/22000_gz_x500_payload` | `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`, and add `22000_gz_x500_payload` under `# [22000, 22999] Reserve for custom models` in that directory's `CMakeLists.txt` |
| `worlds/default_payload_*.sdf` | `PX4-Autopilot/Tools/simulation/gz/worlds/` (early tests, unreferenced) |
| `worlds/default.sdf.wind_plugins.patch` | `git apply` inside `Tools/simulation/gz` (C.3 ApplyLinkWrench world) |
