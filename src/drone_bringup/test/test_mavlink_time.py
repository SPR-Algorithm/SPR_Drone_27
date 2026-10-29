"""Regressions replay actual FC USB TIMESYNC observations, never invented data."""
import json
from pathlib import Path
import unittest

from drone_bringup.mavlink_time import ClockRejected, MavlinkClock


CAPTURE = Path(__file__).parent / 'fixtures/mavlink_timesync_real_20261029.json'


def replay(sequence):
    clock = MavlinkClock()
    observations = []
    for row in sequence['samples']:
        token = clock.request(row['sent_wall_ns'], row['sent_mono_ns'])
        clock.response(row['remote_ns'], token, row['received_wall_ns'], row['received_mono_ns'])
        observations.append(clock.snapshot())
    return clock, observations


class ActualMavlinkClockTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads(CAPTURE.read_text())

    def test_real_usb_clock_skew_requires_affine_mapping(self):
        sequence = self.fixture['sequences'][0]
        clock, _ = replay(sequence)
        self.assertTrue(clock.ready)
        self.assertLess(clock.scale, 0.999)
        self.assertGreater(clock.scale, 0.998)
        self.assertLess(clock.uncertainty_us, 1000)
        self.assertFalse(clock.snapshot()['physical_sync_accuracy_verified'])
        row = sequence['samples'][-1]
        mapped = clock.map_boot_sample(row['remote_ns'] // 1000,
                                       row['received_wall_ns'], row['received_mono_ns'])
        self.assertLess(abs(mapped - row['received_wall_ns']), 1_000_000)

    def test_recorded_host_clock_step_opens_new_segment(self):
        clock, states = replay(self.fixture['sequences'][1])
        self.assertEqual(clock.segment, 1)
        self.assertTrue(clock.ready)
        self.assertTrue(any(not state['sync_ready'] and state['segment_id'] == 1 for state in states))
        self.assertLess(clock.uncertainty_us, 1000)

    def test_unmatched_reply_never_seeds_clock(self):
        row = self.fixture['sequences'][0]['samples'][0]
        clock = MavlinkClock()
        self.assertFalse(clock.response(row['remote_ns'], row['sent_mono_ns'],
                                        row['received_wall_ns'], row['received_mono_ns']))
        self.assertEqual(len(clock.samples), 0)

    def test_epoch_timestamp_cannot_be_mistaken_for_boot_time(self):
        sequence = self.fixture['sequences'][0]
        clock, _ = replay(sequence)
        row = sequence['samples'][-1]
        with self.assertRaisesRegex(ClockRejected, 'unexpected_epoch'):
            clock.map_boot_sample(row['received_wall_ns'] // 1000,
                                  row['received_wall_ns'], row['received_mono_ns'])

    def test_expired_sync_cannot_publish(self):
        clock, _ = replay(self.fixture['sequences'][0])
        row = self.fixture['sequences'][0]['samples'][-1]
        clock.tick(row['received_wall_ns'] + 2_000_000_000,
                   row['received_mono_ns'] + 2_000_000_000)
        self.assertFalse(clock.ready)
        self.assertEqual(clock.reason, 'mavlink_timesync_timeout')


if __name__ == '__main__':
    unittest.main()
