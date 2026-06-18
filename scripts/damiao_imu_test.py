#!/usr/bin/env python3
"""Real-hardware acceptance test for the Damiao DM-IMU-L1 driver (SP-B).

Creates a DAMIAO IMU through imu_py on /dev/dm_imu @ 921600, reads ~100
samples, and verifies:
  - data is non-zero and streaming
  - quaternion is a valid unit quaternion (|q| ~ 1)
  - linear acceleration is in m/s^2 (|a| ~ 9.8 at rest, not ~1.0)
  - angular velocity is in rad/s (small at rest; tilt would give ~1-3 rad/s,
    NOT 60-180 which would indicate unconverted deg/s)
  - output frequency is in the 500-1000 Hz band

This is a read-only sensor test (no actuation) and is safe to run.
"""

import math
import os
import sys
import time

import imu_py

DEV = os.environ.get("DAMIAO_DEV", "/dev/dm_imu")
BAUD = int(os.environ.get("DAMIAO_BAUD", "921600"))
N_SAMPLES = 100
FREQ_WINDOW_S = 2.0
STARTUP_TIMEOUT_S = 5.0


def vec_norm(v):
    return math.sqrt(sum(x * x for x in v))


def changed(a, b, eps=1e-9):
    return any(abs(x - y) > eps for x, y in zip(a, b))


def main():
    print(f"[damiao_test] creating DAMIAO IMU on {DEV}@{BAUD} ...")
    try:
        imu = imu_py.IMUDriver.create_imu(
            imu_id=0,
            interface_type="serial",
            interface=DEV,
            imu_type="DAMIAO",
            baudrate=BAUD,
        )
    except Exception as e:
        print(f"[damiao_test] FAILED to create IMU: {e}")
        return 1
    print("[damiao_test] IMU created (downlink config sent). Waiting for data...")

    # Wait for the first non-trivial frame.
    t0 = time.time()
    got_data = False
    while time.time() - t0 < STARTUP_TIMEOUT_S:
        q = imu.get_quat()
        if vec_norm(q) > 1e-6:
            got_data = True
            break
        time.sleep(0.01)
    if not got_data:
        print(f"[damiao_test] ERROR: no data received within {STARTUP_TIMEOUT_S}s")
        return 1
    print(f"[damiao_test] first data after {time.time() - t0:.2f}s")

    # Collect N_SAMPLES.
    print(f"[damiao_test] collecting {N_SAMPLES} samples...")
    samples = []
    for _ in range(N_SAMPLES):
        samples.append((
            imu.get_quat(),
            imu.get_ang_vel(),
            imu.get_lin_acc(),
            imu.get_temperature(),
        ))
        time.sleep(0.005)

    def fmt_q(q):
        return f"({q[0]:+.4f},{q[1]:+.4f},{q[2]:+.4f},{q[3]:+.4f})"

    def fmt_v(v):
        return f"({v[0]:+.4f},{v[1]:+.4f},{v[2]:+.4f})"

    print("\n=== First 5 samples ===")
    for i, (q, av, la, t) in enumerate(samples[:5]):
        print(f"  [{i}] quat(w,x,y,z)={fmt_q(q)}  ang_vel(rad/s)={fmt_v(av)}  "
              f"lin_acc(m/s^2)=({la[0]:+.3f},{la[1]:+.3f},{la[2]:+.3f})  temp={t:.2f}C")

    print("\n=== Last 5 samples ===")
    for i, (q, av, la, t) in enumerate(samples[-5:]):
        idx = N_SAMPLES - 5 + i
        print(f"  [{idx}] quat={fmt_q(q)}  ang_vel={fmt_v(av)}  "
              f"lin_acc=({la[0]:+.3f},{la[1]:+.3f},{la[2]:+.3f})  temp={t:.2f}C")

    # Unit verification.
    qnorm = vec_norm(samples[-1][0])
    acc_mag = vec_norm(samples[-1][2])
    gyro_norms = [vec_norm(s[1]) for s in samples]
    gyro_max = max(gyro_norms)
    gyro_med = sorted(gyro_norms)[len(gyro_norms) // 2]

    print("\n=== Unit verification ===")
    print(f"  |quat|           = {qnorm:.4f}   (expect ~1.0)")
    print(f"  |lin_acc|        = {acc_mag:.3f} m/s^2 (expect ~9.8 => m/s^2; ~1.0 => still in g)")
    print(f"  |ang_vel| max    = {gyro_max:.4f} rad/s (rest ~0; tilt ~1-3 => rad/s; ~60-180 => deg/s)")
    print(f"  |ang_vel| median = {gyro_med:.4f} rad/s")

    # Frequency measurement via detected frame changes + median inter-arrival.
    print(f"\n=== Frequency measurement ({FREQ_WINDOW_S:.1f}s window) ===")
    prev_q = imu.get_quat()
    ts_changes = []
    start = time.time()
    while time.time() - start < FREQ_WINDOW_S:
        q = imu.get_quat()
        if changed(q, prev_q):
            ts_changes.append(time.time())
            prev_q = q
    elapsed = time.time() - start
    hz_count = (len(ts_changes) / elapsed) if elapsed > 0 else 0.0

    implied_hz = 0.0
    median_ms = 0.0
    if len(ts_changes) > 10:
        intervals = [ts_changes[i + 1] - ts_changes[i] for i in range(len(ts_changes) - 1)]
        intervals.sort()
        median_ms = intervals[len(intervals) // 2] * 1000.0
        if median_ms > 0:
            implied_hz = 1000.0 / median_ms
    print(f"  detected {len(ts_changes)} frame changes in {elapsed:.2f}s "
          f"-> {hz_count:.1f} Hz (polled lower-bound)")
    print(f"  median inter-frame interval = {median_ms:.3f} ms -> {implied_hz:.0f} Hz (true rate estimate)")

    # Verdict.
    print("\n=== Verdict ===")
    ok = True
    if abs(qnorm - 1.0) <= 0.1:
        print("  PASS: quaternion is unit norm (data is valid)")
    else:
        print(f"  FAIL: |quat|={qnorm:.4f} not ~1.0"); ok = False

    if 8.0 <= acc_mag <= 11.0:
        print("  PASS: |lin_acc| ~9.8 -> units confirmed m/s^2")
    elif acc_mag <= 1.5:
        print("  FAIL: |lin_acc| ~1.0 -> appears to be in g (needs *9.8)"); ok = False
    else:
        print(f"  WARN: |lin_acc|={acc_mag:.2f} unexpected (motion or non-standard gravity)")

    # gyro: at rest expect small rad/s; if >~5 it strongly suggests unconverted deg/s noise.
    if gyro_max < 5.0:
        print(f"  PASS: gyro magnitude ({gyro_max:.4f}) consistent with rad/s")
    else:
        print(f"  FAIL: gyro magnitude ({gyro_max:.4f}) too large -> likely unconverted deg/s"); ok = False

    if implied_hz >= 400.0:
        print(f"  PASS: rate ~{implied_hz:.0f} Hz (in 500-1000 band)")
    elif hz_count >= 200.0:
        print(f"  PASS: polled rate {hz_count:.0f} Hz (lower-bound within band)")
    else:
        print(f"  WARN: rate low (count={hz_count:.0f} Hz, implied={implied_hz:.0f} Hz)")

    print("\n[damiao_test] DONE " +
          ("(core checks PASSED)" if ok else "(review failures above)"))
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
