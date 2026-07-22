# Fixed-route slam_toolbox + Smac Hybrid-A* + Nav2 MPPI cruise

This is an independent replacement stack. It does not modify or start the old Cartographer/VP100/`nav2_ackermann_v2` stack.

## Behavior

- Loads `/root/competition/maps/map_slam_20260716_203629.posegraph` in
  slam_toolbox localization mode. Its configured map start is only a
  posegraph-load bootstrap; it cannot release the vehicle.
- Never publishes a fixed race `/initialpose`. The mission remains stopped
  until the operator uses RViz **2D Pose Estimate** in the `map` frame. After
  that message it requires at least three newer map TF samples close to the
  clicked pose (within 0.50 m and 35 deg), then one continuous second of
  healthy scan, fused odom, navigation map, and
  `map -> base_footprint` before creating the mission thread.
- Starts slam_toolbox only after the live `odom_test -> base_footprint` transform
  exists; it does not assume the sensors are ready after a fixed delay.
- Rigidly transforms the fixed field coordinates from the lower-right
  black-wall centerline intersection `(-0.576, -0.177)` into the map frame.
  The fitted field yaw is `-1.918 deg`; it is applied once to every waypoint
  and heading. No coordinate scaling or mirroring is applied.
- Starts the N10 from a stack-owned parameter file: stable USB identity, `/scan`, `laser_frame`, and high-precision LaserScan mode are explicit and do not depend on an installed config copy.
- Starts only after a fresh manual RViz `/initialpose`, followed by stable
  `/scan`, `/odom_fused_test`, `/navigation_map`, and
  `map -> base_footprint`.
- After the initial-pose/health gate, asks Smac Hybrid-A* to connect the
  ordered cruise anchors using a forward-only Dubins model with a 0.30 m
  minimum turning radius. The resulting global path is then sent to
  `/follow_path`, where Ackermann MPPI tracks it and handles scan-to-scan
  corrections.
- Global planning is event-driven rather than periodic: once before WP1, once
  after the reverse arc for WP2-WP14, and only after a blocked/aborted
  controller action or a recovered localization fault. Each Hybrid-A* segment
  has a 0.35 s search limit; the action timeout is `0.75 + 0.35 * segments`
  seconds, capped at 6 s. This keeps localization and MPPI responsive on the
  8-core Cortex-A55 board.
- Both Nav2 obstacle layers consume only the current N10 scan
  (`observation_persistence: 0.0`) and treat infinite no-return ranges as
  valid clearing rays. This prevents old cone endpoints from remaining as
  inflated circular ghost obstacles.
- The rolling local costmap uses only live N10 obstacle data followed by
  inflation; it does not load the map-frame static layer and treats unobserved
  rolling-window cells as free. The global costmap keeps the guarded static
  map for Smac and clears only static cells under the robot's current
  footprint.
- Local and global inflation use a 0.15 m radius and 12.0 cost scale. The
  robot footprint and padding remain unchanged, while separate wall and cone
  soft-cost regions stop merging once their physical gap is about 0.30 m.
- Stops at WP1 and currently skips the camera and `image_upload_analyzer`
  workflow. Once localization is healthy, it proceeds directly to the reverse
  arc. The analysis implementation remains available but is not called by the
  race mission.
- Leaves WP1 with a closed-loop, reverse-only 0.30 m-radius arc (at most
  0.80 m). Reverse MPPI is constrained to `-0.12..-0.11 m/s`, so its linear
  command cannot silently become zero or forward while the car turns toward
  WP2. The reverse action has a hard 8.0 s limit: if alignment is still
  incomplete, cancellation must be accepted and reach a terminal action
  state. The chassis must then provide at least three new, fresh fused-odom
  samples over 0.20 s confirming it is stopped before the continuous route is
  rebuilt from the current pose toward WP2. Either confirmation failing
  latches the software stop.
- Preserves MPPI's body yaw-rate sign in reverse. The Origincar Ackermann
  callback divides its `steering_angle` field by two and sends that value in
  the same serial slot used by `Twist.angular.z`; the adapter therefore sends
  `2 * limited_yaw_rate` in that legacy field. It intentionally does not send
  the physical front-wheel angle, whose sign differs from yaw rate in reverse.
- Uses chassis odometry for planar displacement, but reprojects each encoder
  displacement with the fused odom/IMU heading before integrating
  `/odom_fused_test` x/y. Thus a corrected yaw and the published translation
  describe one internally consistent trajectory.
- Treats WP2-WP13 as ordered soft planning anchors, then stops 5 cm inward
  from field WP14 at `(0.50, 0.25)` (map approximately `(-0.068, 0.056)`),
  still inside WP14's 10 cm acceptance zone. The final controller
  position tolerance is 6 cm and its yaw tolerance is 0.12 rad. The mission
  uses a wall-safe finish region: ±10 cm along the final path and at most
  ±4 cm across it, narrowed further as yaw error increases by checking the
  rotated padded footprint against the measured wall clearance. A near-wall
  stop therefore cannot fall into the old 2-5 cm dead zone or be accepted
  across the wall. An internal unscored point at field `(0.85, 0.25)` aligns
  the final segment with the bottom wall. Smac plans to that exact approach
  pose, then the mission appends a 3 cm-sampled straight segment only after
  checking every padded-footprint sample against `/navigation_map`. The
  same Ackermann global path remains active throughout WP2-WP14; a native Controller
  Server speed limit reduces the final 0.50 m to 0.12 m/s without an
  intermediate stop.
- On a blocked/aborted path, recomputes the remaining ordered route with Smac
  Hybrid-A* instead of searching many re-entry points. It makes at most three
  event-driven attempts and then latches a stop. A `/navigation_map` guard
  converts grey unknown cells and the map perimeter to lethal occupancy, so
  both grey and black remain forbidden.
- Cancels immediately for a localization jump. A stale scan/odom/TF must
  persist for 0.20 seconds before an active path is canceled, avoiding a
  one-callback scheduling spike while still failing closed. It resumes from a
  current-path projection only after one second of stable localization.
- The A55 race profile uses 400 MPPI samples for forward tracking and gives
  Nav2 consumers 0.45 s of TF tolerance. slam_toolbox keeps its own
  `transform_timeout` at 0.30 s and a three-scan queue, so tolerance cannot
  turn into an old-scan backlog.
- Latches the software stop on any unrecoverable reverse maneuver or mission exception; the only reset is restarting the stack.

## Deployment target

Copy this directory to:

```text
/root/competition/nav2_followpath_mppi
```

The selected map requires all four files with the same base name:

```text
map_slam_20260716_203629.posegraph
map_slam_20260716_203629.data
map_slam_20260716_203629.pgm
map_slam_20260716_203629.yaml
```

## Start and stop

Keep the car completely still during the BMI088 startup calibration. The
command below launches the full stack, but the car remains stopped:

```bash
bash /root/competition/nav2_followpath_mppi/start_stack.sh
```

In RViz, set **Fixed Frame** to `map`, choose **2D Pose Estimate**, click the
car's physical position and drag the arrow toward field `+X`. Confirm that the
red scan overlays the black walls. Receipt of that `/initialpose` starts a new
one-second stability window; the mission departs automatically only after that
window passes.

Emergency software stop (latched until the stack is restarted):

```bash
ros2 service call /cruise/stop std_srvs/srv/Trigger '{}'
```

Monitor mission state:

```bash
ros2 topic echo /cruise/status
```

Before the RViz pose is received, the state remains:

```text
WAITING_FOR_RVIZ_INITIALPOSE
```

For a no-hardware configuration inspection after deployment:

```bash
source /opt/ros/humble/setup.bash
python3 /root/competition/nav2_followpath_mppi/validate_config.py --robot
```

Do not run this stack alongside `base_serial.launch.py`, an old cruise node, or another publisher to `/ackermann_cmd`.

The full race launch keeps the camera and the two 1200-pose diagnostic Path
topics off during navigation to protect scan/TF callback latency. The isolated
lidar-localization test enables the Path topics explicitly. The optional
`camera:=true` launch override is for stationary diagnostics with
`mission:=false`; normal race startup leaves it false because the mission owns
the WP1 camera process.

For a race run, use RViz only to publish the manual initial pose. Once
`INITIALPOSE_ACCEPTED_STARTING_MISSION` appears, close the RViz window without
interrupting the launch terminal. RViz is not part of localization or control.

## Verification boundary

Development-time checks are intentionally offline: pure geometry/controller unit tests, Python compilation, YAML parsing, and configuration invariants. No hardware node or chassis command is started while the vehicle is powered off. First powered validation must be done wheels raised or with the physical emergency stop ready.

## Isolated lidar-localization test

The race launch now uses the validated incremental odom/IMU yaw fusion. The
same localization can still be checked independently without any
drive-command publisher:

```bash
bash /root/competition/nav2_followpath_mppi/start_lidar_localization_test.sh
```

See `LIDAR_LOCALIZATION_TEST.md` for its exact update order, covariance values,
TF chain, and RViz acceptance checks.
