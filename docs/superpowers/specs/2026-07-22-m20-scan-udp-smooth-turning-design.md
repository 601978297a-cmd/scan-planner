# M20 SCAN UDP Smooth Turning Design

## Goal

Eliminate the left-right twitch caused by small angular commands and abrupt UDP
yaw-axis reversals while preserving the existing navigation, localization, map,
linear-velocity, auto-ARM, and safety behavior.

## Scope

This change is limited to the SCAN Planner UDP command bridge and its tests.
It does not modify Super-LIO, TF, radar update frequency, grid-map quality,
planner behavior, linear velocity control, or auto-ARM behavior.

## Current Problem

The bridge currently accepts angular velocity commands above `0.01 rad/s`.
The UDP mapper then converts any accepted turn into an axis magnitude of at
least `0.50`. A small change in the sign of `angular.z` can therefore switch
the dog directly from a substantial left-turn command to a substantial
right-turn command, producing a visible side-to-side twitch.

## Proposed Behavior

### Small-command filtering

Increase `yaw_zero_epsilon` from `0.01` to `0.04 rad/s`. Commands inside this
band are treated as zero. Align the yaw mapper's start threshold to `0.04` and
use a `0.02` stop threshold so that noise near zero cannot repeatedly start a
turn, while an active turn can still stop predictably.

Keep `udp_yaw_deadzone` at `0.50`. This value may represent the robot's minimum
effective actuator input, so it will not be lowered without hardware evidence.

### UDP-axis smoothing

Add a yaw-axis slew limiter after angular velocity has been mapped to the UDP
axis. The desired UDP yaw remains the mapper output, but the transmitted yaw
moves toward it by no more than `udp_yaw_slew_rate * dt` per update.

Use an initial `udp_yaw_slew_rate` of `2.0 axis units/s`. At the existing 20 Hz
bridge rate this changes the yaw axis by at most `0.10` per cycle and reaches
the `0.50` minimum effective turn in about 0.25 seconds.

The limiter applies only to UDP yaw. UDP `x` and `y` behavior remains unchanged.

### Direction reversal

When the desired yaw changes sign, the same slew limiter must move the current
yaw through zero before building command in the opposite direction. It must
never jump directly from positive to negative yaw, or vice versa.

### Safety behavior

Safety has priority over smoothing. A health fault, stale command, disarm, or
transition into STOPPING must transmit the existing immediate zero command and
reset both mapper and slew state. The next healthy turn starts again from zero.

## Configuration

Add `udp_yaw_slew_rate: 2.0` to the M20 UDP bridge configuration and update:

- `yaw_zero_epsilon: 0.04`
- `yaw_start_threshold: 0.04`
- `yaw_stop_threshold: 0.02`

All other navigation and actuator parameters remain unchanged.

## Verification

Add unit tests covering:

- small angular commands below `0.04` map to zero;
- yaw ramps without overshoot at the configured rate;
- a left-right reversal crosses zero before changing sign;
- a safety stop bypasses smoothing and sends zero immediately;
