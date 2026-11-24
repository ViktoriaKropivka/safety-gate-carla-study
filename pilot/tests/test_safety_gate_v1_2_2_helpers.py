import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "code" / "carla_two_vehicle_safety_gate_v1_2_2.py"

# The CARLA API is not needed for these pure helper tests.
sys.modules.setdefault("carla", types.ModuleType("carla"))
spec = importlib.util.spec_from_file_location("safety_gate_v122", SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SafetyGateHelperTests(unittest.TestCase):
    def test_half_second_is_exactly_50_ticks(self):
        self.assertEqual(module.delay_to_ticks(0.5, 0.01), 50)

    def test_zero_delay_is_zero_ticks(self):
        self.assertEqual(module.delay_to_ticks(0.0, 0.01), 0)

    def test_non_grid_aligned_delay_is_rejected(self):
        with self.assertRaises(ValueError):
            module.delay_to_ticks(0.505, 0.01)

    def test_unknown_lead_brake_start_becomes_invalid_reason(self):
        profile = [{"t": 0.0, "v": 10.0, "s": 0.0}, {"t": 0.1, "v": 9.0, "s": 0.95}]
        result = module.predict_min_gap_emergency(
            profile, current_gap_m=10.0, ego_speed_mps=10.0,
            lead_brake_age_s=None, lead_speed_mps=10.0,
        )
        self.assertEqual(result[2], "UNKNOWN_LEAD_BRAKE_START_TIME")

    def test_non_numeric_lead_brake_start_becomes_invalid_reason(self):
        profile = [{"t": 0.0, "v": 10.0, "s": 0.0}, {"t": 0.1, "v": 9.0, "s": 0.95}]
        result = module.predict_min_gap_emergency(
            profile, current_gap_m=10.0, ego_speed_mps=10.0,
            lead_brake_age_s="unknown", lead_speed_mps=10.0,
        )
        self.assertEqual(result[2], "UNKNOWN_LEAD_BRAKE_START_TIME")

    def test_profile_validation_bound_loads(self):
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", suffix=".json", delete=False) as f:
            json.dump({"conservative_two_vehicle_gap_error_bound_m": 1.7961018}, f)
            path = f.name
        try:
            bound, reason = module.load_profile_validation(path)
            self.assertAlmostEqual(bound, 1.7961018)
            self.assertEqual(reason, "")
        finally:
            Path(path).unlink(missing_ok=True)

    def test_missing_profile_validation_is_rejected(self):
        bound, reason = module.load_profile_validation("/path/that/does/not/exist.json")
        self.assertIsNone(bound)
        self.assertEqual(reason, "MISSING_PROFILE_VALIDATION_METADATA")

    def test_missing_reference_profile_returns_empty_for_episode_invalid(self):
        self.assertEqual(module.load_reference_profile("/path/that/does/not/exist.csv"), [])


if __name__ == "__main__":
    unittest.main()
