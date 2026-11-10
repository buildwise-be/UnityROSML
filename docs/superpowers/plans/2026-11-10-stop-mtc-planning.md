# Stop MTC planning

**Goal:** Let an operator stop pending MTC computation and retry once ROS releases the planner.

Use MTC's cooperative task preemption. Revoke cancelled results at the commit boundary, even if MTC already found a partial solution. Keep cancellation independent from motion execution and from the main task lock. A running solver or scene service returns at its existing cancellation checkpoint or timeout.

Use the existing fixture request ID and the existing TCP timestamp correlation. For item/speed pick requests, a nonzero pick contact stamp is a correlation token, echoed on planning status and the flattened preview. Contacts continue to resolve in static `ur_base_link`; clear the stamp before pose planning. Preserve zero-stamped legacy behavior and all message definitions.

Expose read-only `mtc_planning_stop_enabled`. Send `std_msgs/String` on `/mtc/cancel_planning` with `scope/request_id`. The scopes are `pick`, `tcp`, and `fixture`. Publish correlated JSON `{key,status,reason}` on `/mtc/planning_control/<scope>`. Statuses are `started`, `stopping`, `cancelled`, `finished`, and `rejected`. Unity sends a stop immediately and waits for a matching terminal status. ROS remembers recently stopped keys so a stop that arrives before planning prevents that exact request from starting. A stale key never preempts a different request. Publish terminal status after releasing an admitted job's task lock.

Use `/execute_pick_place_plan` to send the reviewed plan ID for a correlated pick. Validate the exact ID at backend admission. Keep the existing Boolean Execute command for legacy zero-stamp picks.

## Work

- [x] Add a synchronized backend planning cancellation handle. Register the current task around computation, check cancellation across fixture cycles and scene/setup steps, and seal success atomically with cancellation. Retire cancelled prepared/repeat state and keep existing owned-scene cleanup rules.
- [x] Add shared Unity stop-request tracking and bind it to simple pick, exact TCP, and fixture controllers. Cancel pre-publication checks locally. Keep the backend job token across local review invalidation, filter stale results, and hide cancelled previews immediately.
- [x] Show Stop Planning in the Execute slot in the main task menu and fixture Review page. Enable Plan after the matched terminal status. Preserve operation with older backends via capability detection.
- [x] Check Unity menu behavior, request correlation, Stop after reconnect, and identified pick execution. The focused EditMode suite passed 293 tests. The runtime compile covered 120 main sources and 10 helper sources with zero errors.

The ROS repository carries its own replay procedure and results. The Unity tests
used the actual connector assembly with offline message injection and local
publish callbacks.
