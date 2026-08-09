# scan_cmd_vel_follower

Conservative debug follower for converting SCAN-Planner local paths into
`geometry_msgs/msg/Twist`.

Default behavior:

- subscribes `/visual_local_trajectory`
- subscribes `/Odometry`
- publishes `/scan_cmd_vel_debug`
- disables lateral motion (`linear.y = 0`)
- publishes zero if path or odom is stale

Start:

```bash
ros2 launch scan_cmd_vel_follower scan_cmd_vel_follower.launch.py
```

After direction and stop behavior are verified, the output topic can be changed
to `/cmd_vel`.
