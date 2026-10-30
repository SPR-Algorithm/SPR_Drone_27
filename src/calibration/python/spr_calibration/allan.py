"""Overlapping Allan deviation for measured stationary IMU samples.

For rate measurements, white noise is sigma(tau)=N/sqrt(tau), while
bias Brownian motion is sigma(tau)=K*sqrt(tau/3). We fit those slopes
only where the measured local log slope supports the corresponding model.
"""

import numpy as np

from .common import CalibrationError, finite_positive


def require_allan_sampling(stamps_ns):
    """Apply the same measured-cadence gate before capture and offline fitting.

    This does not establish stationarity, recording duration or noise values.
    Timestamps are neither resampled nor sorted to make a stream pass.
    """
    stamps = np.asarray(stamps_ns, dtype=np.int64)
    if stamps.ndim != 1 or len(stamps) < 100:
        raise CalibrationError("Allan sampling readiness requires >=100 timestamps")
    intervals = np.diff(stamps).astype(float) / 1e9
    if stamps[0] <= 0 or np.any(intervals <= 0):
        raise CalibrationError("Allan sampling timestamps must be positive and strictly increasing")
    period = float(np.median(intervals))
    gap = float(np.max(intervals))
    jitter = float(np.quantile(np.abs(intervals - period), 0.99))
    metrics = {"sample_count": len(stamps), "median_period_s": period,
               "max_gap_s": gap, "max_gap_periods": gap / period,
               "p99_jitter_s": jitter, "p99_jitter_periods": jitter / period}
    if gap > 1.5 * period or jitter > 0.1 * period:
        raise CalibrationError(
            "Allan requires uniform sampling: gaps <=1.5x period, p99 jitter <=10%; "
            f"measured median={period:.9g}s, max_gap={gap:.9g}s ({gap / period:.6g}x), "
            f"p99_jitter={jitter:.9g}s ({100 * jitter / period:.6g}%); do not silently resample")
    return metrics


def overlapping_allan(samples, dt, cluster_sizes=None):
    values = np.asarray(samples, dtype=float)
    if values.ndim == 1:
        values = values[:, None]
    if len(values) < 100 or dt <= 0 or not np.isfinite(values).all():
        raise CalibrationError("Allan deviation requires >=100 finite, regularly sampled measurements")
    if cluster_sizes is None:
        cluster_sizes = np.unique(np.geomspace(1, max(2, len(values) // 20), 80).astype(int))
    cumulative = np.vstack([np.zeros((1, values.shape[1])), np.cumsum(values, axis=0)])
    taus, deviations, counts = [], [], []
    for size in cluster_sizes:
        size = int(size)
        count = len(values) - 2 * size + 1
        if size < 1 or count < 20:
            continue
        difference = (cumulative[2 * size:] - 2 * cumulative[size:-size] + cumulative[:-2 * size]) / size
        deviations.append(np.sqrt(np.mean(difference**2, axis=0) / 2))
        taus.append(size * dt)
        counts.append(count)
    return np.asarray(taus), np.asarray(deviations), np.asarray(counts)


def fit_power_law(tau, deviation, slope, minimum_points=5, slope_tolerance=0.18):
    tau, deviation = np.asarray(tau), np.asarray(deviation)
    valid = (tau > 0) & (deviation > 0) & np.isfinite(deviation)
    tau, deviation = tau[valid], deviation[valid]
    if len(tau) < minimum_points:
        raise CalibrationError("Insufficient positive Allan points")
    x, y = np.log(tau), np.log(deviation)
    windows = []
    for start in range(len(x) - minimum_points + 1):
        for end in range(start + minimum_points, len(x) + 1):
            if x[end - 1] - x[start] < np.log(3):
                continue
            fit = np.polyfit(x[start:end], y[start:end], 1)
            error = float(np.sqrt(np.mean((y[start:end] - np.polyval(fit, x[start:end]))**2)))
            if abs(fit[0] - slope) <= slope_tolerance and error <= 0.15:
                windows.append((x[end - 1] - x[start], -error, start, end, fit))
    if not windows:
        raise CalibrationError(f"No measured Allan region supports slope {slope:+.1f}; record longer or check stationarity")
    _, _, start, end, fit = max(windows, key=lambda value: (value[0], value[1]))
    # Fix the physically specified slope to avoid a tau-dependent coefficient.
    intercept = float(np.mean(y[start:end] - slope * x[start:end]))
    residual = y[start:end] - (intercept + slope * x[start:end])
    coefficient = float(np.exp(intercept) * (np.sqrt(3) if slope == 0.5 else 1))
    return {"coefficient": coefficient, "observed_slope": float(fit[0]),
            "expected_slope": slope, "tau_min_s": float(tau[start]),
            "tau_max_s": float(tau[end - 1]), "points": end - start,
            "log_residual_rms": float(np.sqrt(np.mean(residual**2))),
            "coefficient_log_standard_error": float(np.std(residual) / np.sqrt(end - start))}


def analyze_allan(stamps_ns, values, min_duration_s=3600.0, imu_topic="/spr/imu"):
    finite_positive(min_duration_s, "minimum Allan duration")
    stamps_ns, values = np.asarray(stamps_ns, dtype=np.int64), np.asarray(values, dtype=float)
    if values.shape != (len(stamps_ns), 6) or len(stamps_ns) < 100:
        raise CalibrationError("IMU input requires Nx6 [gyro_xyz,accel_xyz] samples")
    duration = float((stamps_ns[-1] - stamps_ns[0]) / 1e9)
    if duration < min_duration_s:
        raise CalibrationError(f"Static Allan recording is {duration:.1f}s; >= {min_duration_s:.1f}s required")
    sampling = require_allan_sampling(stamps_ns)
    dt = sampling["median_period_s"]
    gyro, accel = values[:, :3], values[:, 3:]
    accel_norm = np.linalg.norm(np.mean(accel, axis=0))
    if not 9.0 <= accel_norm <= 10.6 or np.linalg.norm(np.mean(gyro, axis=0)) > 0.05:
        raise CalibrationError("Static IMU mean is inconsistent with gravity/stationarity; check units and frame")
    # Reject handling shocks and slow platform motion in one-second block means.
    block = max(1, round(1 / dt))
    usable = len(values) // block * block
    means = values[:usable].reshape(-1, block, 6).mean(axis=1)
    if np.max(np.linalg.norm(means[:, :3] - np.median(means[:, :3], axis=0), axis=1)) > 0.1:
        raise CalibrationError("Gyro motion/shock during stationary Allan recording")
    if np.max(np.linalg.norm(means[:, 3:] - np.median(means[:, 3:], axis=0), axis=1)) > 0.5:
        raise CalibrationError("Acceleration motion/shock during stationary Allan recording")
    tau, deviation, counts = overlapping_allan(values, dt)
    fits, failures = {}, []
    for index, axis in enumerate(("gx", "gy", "gz", "ax", "ay", "az")):
        fits[axis] = {}
        for kind, slope in (("white_noise", -0.5), ("bias_random_walk", 0.5)):
            try:
                fits[axis][kind] = fit_power_law(tau, deviation[:, index], slope)
            except CalibrationError as error:
                failures.append(f"{axis}.{kind}: {error}")
    result = {"status": "FAILED" if failures else "PASSED", "duration_s": duration,
              "minimum_duration_s": min_duration_s, "formal_duration": duration >= 3600,
              "update_rate_hz": 1 / dt, "sample_count": len(values),
              "sampling": sampling,
              "mean_acceleration_norm_m_s2": float(accel_norm),
              "fits": fits, "failures": failures,
              "tau_s": tau.tolist(), "allan_deviation": deviation.tolist(),
              "overlapping_difference_counts": counts.tolist(),
              "coefficient_policy": "maximum of three measured axis fits; no defaults",
              "uncertainty_note": "Overlapping points are correlated; fit standard errors are descriptive, not independent confidence intervals"}
    if not failures:
        result["imu"] = {"rostopic": imu_topic, "update_rate": 1 / dt,
                         "gyroscope_noise_density": max(fits[a]["white_noise"]["coefficient"] for a in ("gx", "gy", "gz")),
                         "gyroscope_random_walk": max(fits[a]["bias_random_walk"]["coefficient"] for a in ("gx", "gy", "gz")),
                         "accelerometer_noise_density": max(fits[a]["white_noise"]["coefficient"] for a in ("ax", "ay", "az")),
                         "accelerometer_random_walk": max(fits[a]["bias_random_walk"]["coefficient"] for a in ("ax", "ay", "az"))}
    return result
