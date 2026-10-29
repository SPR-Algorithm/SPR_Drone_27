"""Host-side MAVLink TIMESYNC fit; no PX4 or DDS clock is modified."""
from collections import deque
import math
import statistics


EPOCH_NS = 946684800_000_000_000  # 2000-01-01; PX4 hrt boot time is far smaller.


class ClockRejected(ValueError):
    pass


class MavlinkClock:
    """Fit host = reference_host + scale * (PX4 boot - reference_px4).

    Request identifiers are local monotonic nanoseconds. Only matching responses
    from the selected autopilot may reach this class. The measured uncertainty
    is a software RTT/residual bound, not a physical sensor synchronization proof.
    """
    def __init__(self, min_samples=20, max_uncertainty_us=1000.0,
                 timeout_s=1.0, max_clock_step_s=0.01, window_s=4.0):
        if min_samples < 8 or min(max_uncertainty_us, timeout_s, window_s) <= 0:
            raise ValueError('invalid MAVLink synchronization thresholds')
        self.min_samples = min_samples
        self.max_uncertainty_us = max_uncertainty_us
        self.timeout_ns = round(timeout_s * 1e9)
        self.max_clock_step_ns = round(max_clock_step_s * 1e9)
        self.window_ns = round(window_s * 1e9)
        self.segment = -1
        self.wall_minus_mono = None
        self.reset('waiting_for_mavlink_timesync')

    def reset(self, reason):
        self.segment += 1
        self.reason = reason
        self.pending = {}
        self.samples = deque(maxlen=160)
        self.reference_px4_ns = None
        self.reference_host_ns = None
        self.scale = None
        self.residual_us = None
        self.uncertainty_us = None
        self.rtt_us = None
        self.last_response_mono = None
        self.last_remote_ns = None
        self.ready = False

    def observe_host_clock(self, wall_ns, mono_ns):
        difference = wall_ns - mono_ns
        if (self.wall_minus_mono is not None and
                abs(difference - self.wall_minus_mono) > self.max_clock_step_ns):
            self.reset('host_clock_step')
        self.wall_minus_mono = difference

    def request(self, wall_ns, mono_ns):
        self.observe_host_clock(wall_ns, mono_ns)
        self.pending = {token: sent for token, sent in self.pending.items()
                        if mono_ns - sent[1] < 2_000_000_000}
        self.pending[mono_ns] = (wall_ns, mono_ns)
        return mono_ns

    def response(self, tc1, ts1, wall_ns, mono_ns):
        self.observe_host_clock(wall_ns, mono_ns)
        sent = self.pending.pop(ts1, None)
        if sent is None or tc1 <= 0:
            return False
        if tc1 >= EPOCH_NS:
            self.reset('unexpected_epoch_timesync_clock')
            return False
        rtt = mono_ns - sent[1]
        if not 0 < rtt <= 5_000_000:
            return False
        if self.last_remote_ns is not None and tc1 <= self.last_remote_ns:
            self.reset('px4_clock_restarted_or_reordered')
            return False
        self.last_remote_ns = tc1
        self.last_response_mono = mono_ns
        midpoint = sent[0] + rtt // 2
        self.samples.append((tc1, midpoint, rtt, mono_ns))
        while self.samples and mono_ns - self.samples[0][3] > self.window_ns:
            self.samples.popleft()
        self._fit()
        return True

    def _fit(self):
        self.ready = False
        if len(self.samples) < self.min_samples:
            self.reason = 'collecting_mavlink_timesync'
            return
        # Low RTT measurements constrain asymmetric USB/scheduler delay. Keep
        # enough temporal span to estimate the real clock skew, not just offset.
        keep = max(self.min_samples, len(self.samples) // 2)
        selected = sorted(self.samples, key=lambda item: item[2])[:keep]
        selected.sort()
        if selected[-1][0] - selected[0][0] < 500_000_000:
            self.reason = 'insufficient_clock_fit_span'
            return
        x0, y0 = selected[-1][:2]
        xs = [(row[0] - x0) / 1e9 for row in selected]
        ys = [(row[1] - y0) / 1e9 for row in selected]
        xm, ym = statistics.mean(xs), statistics.mean(ys)
        denom = sum((x - xm) ** 2 for x in xs)
        scale = sum((x - xm) * (y - ym) for x, y in zip(xs, ys)) / denom
        if not math.isfinite(scale) or abs(scale - 1.0) > 0.005:
            self.reason = 'clock_skew_outside_5000ppm'
            return
        y_reference = y0 + round((ym - scale * xm) * 1e9)
        residuals = [row[1] - (y_reference + round(scale * (row[0] - x0)))
                     for row in selected]
        self.reference_px4_ns = x0
        self.reference_host_ns = y_reference
        self.scale = scale
        self.residual_us = max(abs(value) for value in residuals) / 1e3
        self.rtt_us = statistics.median(row[2] for row in selected) / 1e3
        self.uncertainty_us = max(abs(error) + row[2] / 2
                                  for error, row in zip(residuals, selected)) / 1e3
        self.ready = self.uncertainty_us <= self.max_uncertainty_us
        self.reason = 'synchronized' if self.ready else 'mavlink_clock_uncertainty_exceeds_budget'

    def tick(self, wall_ns, mono_ns):
        self.observe_host_clock(wall_ns, mono_ns)
        if (self.last_response_mono is not None and
                mono_ns - self.last_response_mono > self.timeout_ns):
            self.reset('mavlink_timesync_timeout')

    def map_boot_sample(self, boot_us, wall_ns, mono_ns):
        self.tick(wall_ns, mono_ns)
        if not 0 < boot_us * 1000 < EPOCH_NS:
            raise ClockRejected('unexpected_epoch_or_zero_highres_timestamp')
        if not self.ready:
            raise ClockRejected(self.reason)
        mapped = self.reference_host_ns + round(self.scale * (boot_us * 1000 - self.reference_px4_ns))
        age = wall_ns - mapped
        if not -10_000_000 <= age <= 200_000_000:
            raise ClockRejected('highres_timestamp_not_near_host_time')
        return mapped

    def snapshot(self):
        return {
            'sync_ready': self.ready, 'sync_reason': self.reason,
            'sync_samples': len(self.samples), 'segment_id': self.segment,
            'reference_px4_ns': self.reference_px4_ns,
            'reference_host_ns': self.reference_host_ns,
            'clock_scale': self.scale,
            'skew_ppm': None if self.scale is None else (self.scale - 1.0) * 1e6,
            'clock_fit_residual_us': self.residual_us,
            'round_trip_time_us': self.rtt_us,
            'sync_uncertainty_us': self.uncertainty_us,
            'sync_budget_us': self.max_uncertainty_us,
            'host_minus_px4_offset_us': (None if self.reference_host_ns is None else
                (self.reference_host_ns - self.reference_px4_ns) / 1e3),
            'sync_protocol': 'mavlink_timesync',
            'timestamp_mode': 'mavlink_boot_to_host',
            'clock_fit_policy': 'affine_lowest_rtt_half_window_min_20_samples',
            'uncertainty_scope': 'software_rtt_and_fit_residual_only',
            'physical_sync_accuracy_verified': False,
        }
