# Live UR16e force / torque

Both task scenes show a **UR16e force / torque** card beside the task menu.
It starts when the task menu starts and uses the existing `ROSService` connection
(initially `10.200.101.144:10000`). No scene wiring or extra Windows installation
is required. Welcome does not start a new sensor subscription.

`ForceTorqueSubscriber` subscribes to
`/force_torque_sensor_broadcaster/wrench` as `geometry_msgs/msg/WrenchStamped`.
This topic, its active controller, and frame `ur_tool0` were confirmed against
the deployed Jazzy driver on 3 November 2026. The configured update rate is
100 Hz. The UI refreshes at 10 Hz while the subscriber accepts every delivered
sample. The installed ROS-TCP endpoint's reliable/volatile subscription is
compatible with the inspected reliable/transient-local publisher.

The card has two gauges: total force magnitude in newtons and total torque
magnitude in newton-metres. Each is the Euclidean norm of the three ROS
components, so the gauge values are always nonnegative. The source frame is
shown below them. Values reflect the driver's current compensation and sensor
zero; torque is measured about the source frame's origin. The readout does not
zero the sensor or send robot commands.

Each gauge has an amber execution-peak marker and a numeric peak value. There
is no marker initially. A deliberate new **Plan** clears both markers, even if
validation later rejects that planning request. An accepted **Execute** also
clears both; only valid samples arriving during that execution establish and
raise the peaks. Repeated pending requests and duplicate status messages do not
reset them. Peaks are calculated for every received sample, independently of
the slower display refresh. The maxima freeze when the execution reaches its
terminal outcome and remain until the next Plan or Execute. Unknown execution
outcomes continue recording while the existing execution guard remains busy.
Target edits, stale telemetry and temporary component disabling preserve the
held maxima. Re-enabling detects any Plan/Execute that occurred while disabled.

Gauge ranges are fixed at 0–100 N and 0–20 Nm by default and are configurable
on the menu in the Inspector. Readings above the selected maximum keep their
actual numeric values and uncapped execution peaks; only the live fill and
peak marker stop at the end of the gauge. The scales indicate magnitude, not
safety thresholds.

**Live** requires a valid sample received locally within the last second and
an available transport. Waiting, disconnected, disabled, malformed, and stale
states show dashes instead of previous live readings and hide the live fill;
historical execution markers remain visible. Freshness uses local
receipt time, independently of clock differences between the PC and ROS host;
it does not measure end-to-end transport latency. Samples preserve their ROS
timestamp in `StampSeconds` / `StampNanoseconds`. Suspension, disabling the
subscriber, and an observed transport loss clear sample readiness. Subscriptions
are removed when leaving the owning workflow scene.

The topic and maximum sample age are Inspector fields. To configure another
driver topic, add `ForceTorqueSubscriber` to the same object as
`ToolSelectionMenu` before running and set its topic. Otherwise the menu adds
the default subscriber at runtime. Existing menus can also use the component's
`IsFresh`, numeric properties, `StatusText`, and `ReadingText`.

For passive backend diagnosis:

```sh
ros2 control list_controllers
ros2 topic info /force_torque_sensor_broadcaster/wrench --verbose
ros2 topic echo /force_torque_sensor_broadcaster/wrench --once
```

These commands only observe the stream. A stopped broadcaster needs ROS-side
attention; Unity does not activate controllers automatically. Driver releases
can use different topic names, so inspect the actual deployment before changing
the default. See the [UR Jazzy controller documentation](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/jazzy/doc/ur_robot_driver/ur_robot_driver/doc/usage/controllers.html).

Offline checks use `prepare_force_validation.py` and
`ForceTorqueSubscriberTests`. Evidence from the read-only endpoint probe is in
`ForceTelemetryValidation/live_endpoint.json`. Neither offline tests nor a TCP
probe verify headset display or a packaged application installation.

The peak-lifecycle change passed 136 gauge/planning/execution checks. The fixed
100 N / 20 Nm range update passed all 46 focused gauge checks and runtime
compilation with zero errors; 14 menu/state previews were refreshed with
text-overflow checks. See the
[fixed-range validation](ForceTelemetryValidation/range_validation_summary.json).
The Windows-to-endpoint probe received 250 valid samples at 97.8 Hz, with
strictly advancing source timestamps at 100.0 Hz. See
[validation evidence](ForceTelemetryValidation/validation_summary.json) and the
[gauge and peak preview](ForceTelemetryValidation/SimplePickAndPlace-peak.png); preview
numbers are synthetic test values.
