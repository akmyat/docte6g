import unittest

from sionnart import SionnaRT


def make_rt(speed=0.0, direction_dot=1.0):
    rt = SionnaRT.__new__(SionnaRT)
    rt.coherence_alpha = 0.4
    rt.coherence_tx_num_cols = 8
    rt.virtual_position_min_separation = 0.5
    rt.cache_threshold_buffer = 1.1
    rt.adaptive_future_horizon_seconds = 3.0
    rt.adaptive_future_min_benefit_seconds = 1.0
    rt.adaptive_future_max_steps = 3
    rt.adaptive_future_direction_dot_threshold = 0.7
    rt.rx_speeds = {"Rx1": speed}
    rt.rx_last_directions = {"Rx1": [1.0, 0.0, 0.0]}
    rt.rx_direction_dots = {"Rx1": direction_dot}
    rt._nearest_transmitter_distance = lambda _point: 10.0
    return rt


class AdaptivePredictionTest(unittest.TestCase):
    def test_zero_speed_creates_no_predictions(self):
        rt = make_rt(speed=0.0)
        self.assertEqual(rt._adaptive_virtual_rx_positions("Rx1", [0.0, 0.0, 1.5]), [])

    def test_slow_speed_skips_when_current_cache_lifetime_is_long(self):
        rt = make_rt(speed=0.1)
        self.assertEqual(rt._adaptive_virtual_rx_positions("Rx1", [0.0, 0.0, 1.5]), [])

    def test_fast_stable_direction_creates_bounded_forward_predictions(self):
        rt = make_rt(speed=2.0)
        self.assertEqual(
            rt._adaptive_virtual_rx_positions("Rx1", [0.0, 0.0, 1.5]),
            [[0.55, 0.0, 1.5], [1.1, 0.0, 1.5], [1.65, 0.0, 1.5]],
        )

    def test_unstable_direction_creates_no_predictions(self):
        rt = make_rt(speed=2.0, direction_dot=0.0)
        self.assertEqual(rt._adaptive_virtual_rx_positions("Rx1", [0.0, 0.0, 1.5]), [])


if __name__ == "__main__":
    unittest.main()
