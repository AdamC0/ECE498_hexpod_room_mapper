import unittest

from controller_status import diagnose_runtime


def ready_snapshot(**updates):
    values = {
        "state": "SEARCH",
        "phase": "go",
        "paused": False,
        "manual_cmd": None,
        "commanded_moving": False,
        "last_cmd": ("stop",),
        "drive": True,
        "use_map": True,
        "use_depth": True,
        "use_depth_map": True,
        "scan_stamp": 100.0,
        "depth_stamp": 100.0,
        "registered_depth_stamp": 100.0,
        "pose_stamp": 100.0,
        "pose_available": True,
        "slam_stale": 2.5,
        "map_trusted": True,
        "have_map": True,
        "map_fresh": True,
        "automatic_reset_pending": False,
        "pose_guard_start": 0.0,
        "floor_ready": True,
        "depth_health": "ready",
        "path_length": 2,
        "web_message": "following frontier route",
    }
    values.update(updates)
    return values


class RuntimeStatusTests(unittest.TestCase):
    def test_pause_is_the_direct_operator_reason(self):
        status = diagnose_runtime(
            ready_snapshot(paused=True, registered_depth_stamp=0.0,
                           web_message="Autonomy paused; manual drive available"), 100.0)
        self.assertEqual(status.code, "AUTO_PAUSED")
        self.assertIn("Autonomy paused", status.line())

    def test_stale_planner_depth_identifies_the_blocking_stream(self):
        status = diagnose_runtime(ready_snapshot(registered_depth_stamp=98.5), 100.0)
        self.assertEqual(status.code, "REGISTERED_DEPTH_STALE")
        self.assertIn("older than 1.0 s", status.summary)

    def test_map_failures_are_reported_separately(self):
        self.assertEqual(
            diagnose_runtime(ready_snapshot(map_trusted=False), 100.0).code,
            "MAP_UNTRUSTED")
        self.assertEqual(
            diagnose_runtime(ready_snapshot(have_map=False), 100.0).code,
            "MAP_UNAVAILABLE")
        self.assertEqual(
            diagnose_runtime(ready_snapshot(map_fresh=False), 100.0).code,
            "MAP_STALE")

    def test_active_command_is_not_reported_as_a_stop(self):
        status = diagnose_runtime(
            ready_snapshot(commanded_moving=True, last_cmd=("forward",)), 100.0)
        self.assertTrue(status.moving)
        self.assertEqual(status.code, "MOTION_ACTIVE")
        self.assertIn("Current action", status.line())


if __name__ == "__main__":
    unittest.main()
