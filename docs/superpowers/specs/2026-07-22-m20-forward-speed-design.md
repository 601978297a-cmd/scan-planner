# M20 Forward Speed Design

## Goal

Allow the navigation controller's configured forward command range to pass through the UDP safety bridge up to 0.15 m/s.

## Change

Change only `max_vx` in `m20_scan_udp_bridge.yaml` from 0.05 to 0.15. Keep `udp_max_x` at 0.50, `max_ax` at 0.10, and all turning and safety freshness settings unchanged.

## Safety and verification

Disarm UDP output before restarting the bridge. After restart, confirm the active `max_vx` is 0.15, the bridge remains stopped until its freshness checks pass, and zero input produces zero UDP preview. Real walking speed remains bounded by `udp_max_x: 0.50`.

## Rollback

Restore `max_vx` to 0.05 and restart the UDP bridge.
