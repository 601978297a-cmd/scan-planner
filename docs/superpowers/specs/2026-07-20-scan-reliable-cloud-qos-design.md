# SCAN Reliable Cloud QoS Design

## Goal

Make the front lidar path reliable from the existing driver publisher through the C++ sensor pose adapter to SCAN Planner. Reduce missing full-size point-cloud samples and keep the existing 0.30-second safety freshness threshold unchanged.

## Evidence

- The lidar driver publishes `/rslidar_points_front` with `RELIABLE`, volatile, `KeepLast(5)`.
- The C++ adapter currently subscribes with sensor-data QoS, which is `BEST_EFFORT`.
- A 15-second comparison received 128 headers with a RELIABLE reader and 73 with a BEST_EFFORT reader.
- Each raw front cloud is approximately 0.63–0.92 MB.
- Fast DDS 2.6.11 shared-memory transport is active between the driver and adapter.
- `/dev/shm` has 7.7 GB free and is only 1% used.
- The driver and adapter UDP sockets currently report zero kernel drops; a 10-second system sample added no UDP receive or send buffer errors.

These results identify reader reliability as the first change to test. They do not justify changing the lidar driver, system socket settings, or global Fast DDS XML.

## Scope

Change only:

- `m20_sensor_pose_adapter_cpp`
- SCAN Planner's lidar cloud/pose message-filter subscriptions
- focused tests that assert the intended QoS wiring

Do not change:

- lidar driver source or configuration
- SuperLIO or relocation
- global Fast DDS XML or kernel settings
- point-cloud contents, rate, filtering, or timestamps
- safety timeouts, command output, or UDP arming state
- the Python adapter fallback

## QoS Design

Use one point-cloud-pair QoS profile:

- reliability: `RELIABLE`
- durability: `VOLATILE`
- history: `KEEP_LAST`
- depth: `5`

Apply it to:

1. The C++ adapter's `/rslidar_points_front` subscription.
2. The C++ adapter's `/scan/front_cloud_synced` publisher.
3. The C++ adapter's `/scan/sensor_pose` publisher because it is timestamp-paired with the cloud.
4. SCAN Planner's synchronized cloud subscriber.
5. SCAN Planner's synchronized sensor-pose subscriber.

Keep `/scan/front_cloud_stamp` on sensor-data QoS. It is a small header message, and changing both Python safety bridges is unnecessary for this point-cloud transport fix.

Depth five matches the existing driver writer history and bounds retained point-cloud memory to approximately five frames per endpoint. RELIABLE allows Fast DDS to repair missing fragments instead of discarding the entire point-cloud sample.

## Data Flow

1. The existing driver publishes a reliable front cloud.
2. The adapter receives the complete sample reliably and queues it without copying its data buffer.
3. The adapter waits for the exact TF timestamp as before.
4. The adapter publishes the matched cloud and sensor pose using the reliable pair profile.
5. SCAN Planner receives both messages reliably and its existing message filter synchronizes them by timestamp.

No TF lookup, queue timeout, FIFO, or publication-order behavior changes.

## Failure Behavior

- A missing DDS fragment is repaired while the sample remains in the five-entry writer history.
- If a subscriber cannot keep up beyond the bounded history, old samples may still be replaced; the design does not allow unbounded backlog.
- Existing adapter queue-full and TF-wait-timeout warnings remain unchanged.
- If runtime latency or CPU increases materially, revert the implementation commit or launch the retained Python fallback while diagnosing.

## Verification

Before restart:

- Build the adapter and planner packages.
- Run all current bringup tests plus focused QoS wiring tests.
- Check `git diff --check`.

Runtime verification:

- Keep UDP output `DISARMED`.
- Confirm driver, adapter input, adapter matched outputs, and planner pair subscriptions report RELIABLE QoS.
- Confirm `/rslidar_points_front` still has one permanent consumer.
- Wait for `/lio/robo/odom` to resume before measuring matched outputs.
- Record `/scan/front_cloud_stamp` for 30 seconds and calculate message rate, maximum gap, and gaps above 0.30 seconds.
- Check adapter CPU, TF-pending warnings, queue drops, and both safety statuses.

Success target:

- matched stamp rate close to the approximately 10 Hz driver rate
- no adapter queue drops
- maximum observed stamp gap below 0.30 seconds during the test window
- no command publisher armed or motion command emitted

## Rollback

Revert the QoS implementation commit. The driver and global middleware configuration remain untouched, so rollback requires no system cleanup or driver rebuild.
