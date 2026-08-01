# Known Issues

Live list of things verified broken or unverified. Update as they're resolved.

## Open

### 1. Merged map geometry is wrong (Phase 2) — **blocking Phase 3**

The full pipeline is proven end to end — Gazebo lidar → `pointcloud_to_laserscan` →
`slam_toolbox` → `/robot_0/map` → `adapter_sim` → backend → PNG — but the resulting
merged grid does not look like the world. It renders as a wedge of free space rather
than the room layout, with hatched/dithered edges instead of clean walls.

What has been ruled out:

- SLAM is running and publishing (`/robot_0/map`, 433×377 @ 0.05 m).
- `adapter_sim` uploads it and the backend ingests it (free≈204k, occupied≈5.9k cells).
- Grid offset arithmetic does not overflow the 800×800 merged grid.
- Narrowing the height band to the lidar frame (`-0.15…0.15`) changed almost nothing,
  which weakens the "wrong height band" hypothesis.

Still to check, roughly in order:

1. **Y-flip consistency.** `MapService.as_png` now applies `np.flipud` so the PNG
   matches the frontend's `worldToGrid` (`gy = height - (y - origin_y)/res`).
   `take_patch` does **not** flip — verify the patch path and the PNG path agree,
   or the GUI will show patches mirrored against the base map.
2. **`MapService._remerge` offset sign.** Confirm against a hand-computed case with a
   non-zero `start_pose`; the tests only cover identity transforms.
3. **Lidar vertical FOV.** ±0.26 rad over 16 m is ±4.2 m of vertical spread. Consider
   narrowing the SDF `<vertical>` range, or using a single ring for 2D mapping.
4. **`slam_toolbox` scan-matching quality** — check `minimum_travel_distance` and
   whether odometry drift is being corrected at all.

A useful next step is bypassing SLAM entirely: publish a synthetic `OccupancyGrid`
with known contents through `adapter_sim` and confirm it renders correctly. That
isolates the map service from the SLAM front end.

### 2. Frontend never visually confirmed

Builds clean and `svelte-check` reports 0 errors, but Chrome in this environment
cannot reach the dev server (`localhost`, `127.0.0.1`, and the LAN IP all fail), so
the layout has never been seen rendered. Open `http://localhost:5173/?mock=1&robots=4`
and check before trusting it.

### 3. Video pipeline unbuilt

`swarmdeck_media` is an empty package and MediaMTX is not installed. `CameraPanel`
degrades to a "no signal" placeholder, which *is* verified. Phase 4.

### 4. `multirobot_map_merge` not installed

`auto` merge mode has no backend. Only `static` mode works. Needs a source build of
`m-explore-ros2`. Phase 5.

## Resolved

### `slam_toolbox` silently does nothing (fixed)

Two traps, both silent — no error, no log line, node appears healthy:

1. It is a **lifecycle node**. On Jazzy it sits in `unconfigured` with zero
   subscribers, logging nothing at all. `use_lifecycle_manager: false` did *not* make
   it self-transition. Fixed by adding `nav2_lifecycle_manager` with `autostart: true`
   in `slam.launch.py`.
2. The **`scan_topic` parameter has no effect** — the topic must be remapped
   (`-r scan:=/robot_0/scan`). With the parameter alone, subscription count stays 0.

### Sim-time mismatch (fixed)

Gazebo stamps sensors with sim time. Without `/clock` bridged and
`use_sim_time:=true` on every node, TF lookups never resolve and SLAM silently
stalls. The bridge and the parameter are now in `slam.launch.py`.

### `angle_increment` too fine (fixed)

Set to 0.0087 rad against a lidar with 360 horizontal samples, producing `inf` in
every other bin. Must be 2π/360 = 0.017453.

## Process hygiene

`pkill -f <pattern>` matches the agent's own shell command line and kills the calling
shell (observed as exit code 144). Use `pkill -x <exe>`, or collect PIDs with
`ps aux | grep '[x]yz'` and kill by PID. Orphaned `gz sim` processes hold DDS ports
and silently poison the next run.
