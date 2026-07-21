# SCAN UDP Auto-Arm with Manual Disarm Latch

## Goal

Remove the need to manually arm the M20 UDP safety bridge after every startup or recoverable sensor interruption, without allowing an old navigation target or trajectory to resume automatically.

## Operator Behavior

- The bridge still starts in `DISARMED` and sends no motion command.
- When all existing arm checks remain healthy for 2.0 continuous seconds, the bridge automatically enters `ARMED`.
- A safety fault immediately follows the existing stop sequence: publish navigation disabled, clear the Planner target and controller trajectory, and send the configured zero-UDP sequence.
- After an automatic safety stop, the bridge automatically arms again only after all checks have remained healthy for 2.0 continuous seconds.
- Automatic re-arm never restores the previous route. The operator must submit a new RViz target after `/scan/navigation_enabled` becomes true.
- A manual DISARM sets a latch. While latched, health recovery cannot automatically arm the bridge.
- The manual latch is cleared only by a successful explicit ARM request or by restarting the bridge. Restarting represents a new operator session and again permits automatic arm after the health-stability delay.

## Configuration

Add two bridge parameters:

- `auto_arm_enabled`: defaults to `false` in code for compatibility and is set to `true` in the M20 UDP configuration.
- `auto_arm_stable_sec`: defaults to `2.0` seconds.

Dry-run mode cannot auto-arm because `enable_udp_output=false` remains an arm blocker.

## State and Data Flow

The existing `DISARMED`, `ARMED`, and `STOPPING` states remain unchanged. The bridge adds:

- `manual_disarm_latched`: initialized false, set true by every manual DISARM request.
- `auto_arm_ready_since`: the monotonic time at which all arm blockers first became empty.

While `DISARMED`, the periodic bridge callback evaluates the same blockers used by the manual ARM service. A blocker resets `auto_arm_ready_since`. When no blocker has existed for the configured stable interval, the bridge uses the same common arm helper as the service.

Manual ARM retains its current safety checks. A successful manual ARM clears the manual-disarm latch. A failed manual ARM leaves the latch unchanged, preventing a failed request from causing a later unexpected automatic arm.

The bridge continues publishing `/scan/navigation_enabled` with reliable, transient-local QoS:

- `false` at startup and at the beginning of every stop.
- `true` only after either manual or automatic arm succeeds.

The Planner and controller behavior introduced in `b356ae1` is unchanged: `false` clears target, B-spline, and navigation markers; `true` waits for a fresh target and trajectory.

## Fault Handling

- Command, odometry, sensor-pose, cloud, timestamp-pair, heartbeat, UDP, or conflict failures still produce zero output immediately and begin the stop sequence.
- Recovering from a fault can only restore the armed gate; it cannot create a motion command because the previous Planner target and controller trajectory were cleared.
- Manual DISARM always wins over automatic behavior.
- Destroying or restarting the bridge still publishes navigation disabled and sends the existing zero sequence when applicable.

## Diagnostics

Add diagnostic fields for:

- `auto_arm_enabled`
- `manual_disarm_latched`
- `auto_arm_ready_age_sec`

Log one message when the stability window begins, when a blocker cancels it, and when automatic arm succeeds. Avoid per-tick log spam.

## Testing

Unit tests will verify:

- startup does not arm before the complete stability interval;
- startup arms after 2.0 continuous healthy seconds;
- a blocker resets the stability interval;
- an automatic fault stop can auto-arm after recovery;
- manual DISARM latches and prevents auto-arm;
- failed manual ARM does not clear the latch;
- successful manual ARM clears the latch;
- dry-run mode never auto-arms;
- navigation-enabled publication still follows successful arm and every stop.

After unit tests, run the complete Pytest suite, Colcon tests, and a full build. Deployment validation will start with real UDP enabled but remain zero-output: confirm a single bridge instance, observe automatic arm only after healthy stability, manually DISARM, and verify that it remains latched despite continued healthy inputs. No navigation goal is required for this validation.
