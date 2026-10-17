"""Evidence checks for post-cancellation robot stop verification."""

from __future__ import annotations

import math


def validate_stop_samples(
    samples,
    *,
    min_samples=5,
    max_receive_gap=0.5,
    max_speed=0.02,
    max_drift=0.03,
    max_yaw_drift=0.03,
):
    """Reject stale, discontinuous, invalid, moving, or rotating odometry.

    A sample is ``(received_monotonic, (x, y), speed, (sec, nanosec), yaw, valid)``.
    """
    if len(samples) < min_samples:
        raise ValueError(f"too few fresh odometry samples during settling: {len(samples)}")
    if any(not sample[5] for sample in samples):
        raise ValueError("invalid or repeated-timestamp odometry during stop verification")
    if any(not all(math.isfinite(value) for value in (*sample[1], sample[2], sample[4]))
           for sample in samples):
        raise ValueError("non-finite odometry during stop verification")

    stamp_ns = [sample[3][0] * 1_000_000_000 + sample[3][1] for sample in samples]
    if any(later <= earlier for earlier, later in zip(stamp_ns, stamp_ns[1:])):
        raise ValueError("odometry message timestamps did not increase during stop verification")
    receive_times = [sample[0] for sample in samples]
    if any(later - earlier > max_receive_gap
           for earlier, later in zip(receive_times, receive_times[1:])):
        raise ValueError(f"odometry stream had a gap over {max_receive_gap:.1f}s during stop verification")
    if any(sample[2] >= max_speed for sample in samples):
        raise ValueError("robot speed rose above stop threshold during settling window")

    first_pose = samples[0][1]
    drift = max(math.hypot(sample[1][0] - first_pose[0], sample[1][1] - first_pose[1])
                for sample in samples)
    if drift > max_drift:
        raise ValueError(f"robot drifted {drift:.3f} m after cancellation")
    first_yaw = samples[0][4]
    yaw_drift = max(abs(math.atan2(math.sin(sample[4] - first_yaw),
                                   math.cos(sample[4] - first_yaw)))
                    for sample in samples)
    if yaw_drift > max_yaw_drift:
        raise ValueError(f"robot rotated {yaw_drift:.3f} rad after cancellation")
    return drift, yaw_drift
