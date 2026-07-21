# Q2 RViz / AMCL diagnosis

Updated: 2026-07-22

## RViz result

![Q2 RViz racing-line and AMCL result](q2_rviz_result.png)

The blue line is `/car_path`, the green/red line is the optimal racing line, and
the white points are `/scan`. Use the scan-to-map alignment—not the blue line
alone—to decide whether controller tuning results are valid.

## Symptoms

- The blue `/car_path` leaves the green/red racing line by a large distance.
- White `/scan` points do not overlap the map walls and may appear outside the map.
- Q2 falsely detects walls as obstacles, takes unusual avoidance paths, or appears slow.

## Root cause

Do not tune Stanley steering while the LiDAR scan and map are misaligned. Q2 converts
LiDAR detections into `map` coordinates using AMCL. A wrong pose therefore corrupts
both racing-line tracking and obstacle classification.

Use this order:

1. Synchronize the laptop and Jetson clocks.
2. Align the LiDAR and map with RViz `2D Pose Estimate`.
3. Save the verified pose with `./race.sh savepose`.
4. Restart in box mode and confirm that alignment persists.
5. Tune steering or speed only after localization is correct.

## Safe alignment procedure

Start without allowing the wheels to drive:

```bash
cd ~/yax
./race.sh stop
./race.sh box q2
```

In RViz:

1. Set `Fixed Frame` to `map`.
2. Select `2D Pose Estimate`.
3. Click the car's real position and drag in its real forward direction.
4. Repeat until the white LiDAR points overlap the map walls.

Save the aligned pose from another Jetson terminal:

```bash
cd ~/yax
./race.sh savepose
```

Restart and verify that the alignment persists:

```bash
./race.sh stop
ros2 daemon stop
./race.sh box q2
```

Only start `./race.sh q2` when the vehicle marker is on the gray track, its heading
matches the physical car, and the white scan overlaps the inner and outer walls.

## Clock synchronization

Compare epoch time on both machines:

```bash
date +%s
ssh jetson@192.168.45.246 date +%s
```

If they differ substantially, copy the laptop time to the Jetson:

```bash
JETSON_TIME="$(date '+%Y-%m-%d %H:%M:%S')"
ssh -t jetson@192.168.45.246 "sudo date -s '$JETSON_TIME'"
```

Restart ROS processes on both machines after changing the clock so stale TF and
message timestamps are removed.

## Log interpretation

- `Message Filter dropping message` only during startup can occur before TF is ready.
  If it continues while driving, check clock sync and `map -> odom -> base_link -> laser`.
- `rcl_shutdown already called` and `publisher's context is invalid` after Ctrl+C are
  shutdown warnings, not the cause of bad driving.
- Q2 may briefly leave the racing line to avoid a real obstacle. Large deviations in
  obstacle-free sections should be treated as a localization problem first.

## Important rule

Never judge controller tuning from an RViz capture where the white LiDAR scan does not
match the map. A controller cannot follow a map-frame racing line from an invalid pose.
