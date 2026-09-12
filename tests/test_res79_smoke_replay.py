#!/usr/bin/env python3
"""RES-79 unit tests for tools/res79_smoke_replay.py (fast, no physics).

Covers the fail-closed pure logic: frame-map derivation, sample/time
conversions, control-boundary math, ASS formatting, gate plumbing, and the
read-only enforcement helper contract. Full replay/render is exercised by the
pipeline's own identity gates, not here.
"""

import json
import sys
import unittest
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))

import res79_smoke_replay as R


class TestFrameMap(unittest.TestCase):
    def test_derived_count_not_hardcoded(self):
        fm = R.build_frame_map()
        # floor(T_END*FPS) + 1, derived from authority constants.
        import math
        want = math.floor(R.T_END * R.FPS) + 1
        self.assertEqual(len(fm), want)
        self.assertGreater(len(fm), 900)
        self.assertLess(len(fm), 930)

    def test_first_last_frames(self):
        fm = R.build_frame_map()
        self.assertEqual(fm[0]["frame"], 0)
        self.assertEqual(fm[0]["sample"], 0)  # t=0 -> nearest is sample 0
        last = fm[-1]
        self.assertAlmostEqual(last["t_frame"], last["frame"] / R.FPS)
        self.assertLessEqual(abs(last["err_s"]), R.PHYS_DT)
        self.assertLess(last["t_frame"], R.T_END)

    def test_monotonic_and_bounded(self):
        fm = R.build_frame_map()
        seq = [f["sample"] for f in fm]
        self.assertTrue(all(0 <= s < R.N_PHYS for s in seq))
        self.assertTrue(all(b >= a for a, b in zip(seq, seq[1:])))
        # Frame 0 at t=0 clips to sample 0 (err=+DT); all others <= DT/2.
        self.assertAlmostEqual(fm[0]["err_s"], R.PHYS_DT)
        for f in fm[1:]:
            self.assertLessEqual(abs(f["err_s"]), R.PHYS_DT / 2 + 1e-12)

    def test_physics_index_for_time(self):
        self.assertEqual(R.physics_index_for_time(0.000125), 0)
        self.assertEqual(R.physics_index_for_time(0.0), 0)
        self.assertEqual(R.physics_index_for_time(0.875), 6999)
        self.assertEqual(R.physics_index_for_time(1e9), R.N_PHYS - 1)


class TestControlMath(unittest.TestCase):
    def test_starts_and_lookup(self):
        sub = np.array([40, 40, 28, 40], dtype=np.int64)
        st = R.control_starts(sub)
        self.assertEqual(list(st), [0, 40, 80, 108])
        self.assertEqual(R.control_index_for_sample(0, st), 0)
        self.assertEqual(R.control_index_for_sample(39, st), 0)
        self.assertEqual(R.control_index_for_sample(40, st), 1)
        self.assertEqual(R.control_index_for_sample(107, st), 2)
        self.assertEqual(R.control_index_for_sample(108, st), 3)

    def test_boundary_index(self):
        sub = np.array([40, 40, 28], dtype=np.int64)
        self.assertEqual(R.boundary_state_index(0, sub), -1)
        self.assertEqual(R.boundary_state_index(1, sub), 39)
        self.assertEqual(R.boundary_state_index(2, sub), 79)


class TestAss(unittest.TestCase):
    def test_escape(self):
        self.assertEqual(R.ass_escape("a{b}\\c"), "a\\{b\\}\\\\c")

    def test_timestamp(self):
        self.assertEqual(R.ass_timestamp(0), "0:00:00.00")
        self.assertEqual(R.ass_timestamp(61.5), "0:01:01.50")
        self.assertEqual(R.ass_timestamp(3661.0), "1:01:01.00")

    def test_ass_head_format_and_box(self):
        head = R._ass_head("t")
        self.assertIn("Format: Layer, Start, End, Style, Name, MarginL, "
                      "MarginR, MarginV, Effect, Text", head)
        self.assertIn(",1,7,3,1,0,", head)  # opaque box style

    def test_ass_single_dialogue_shape(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "s.ass"
            R.build_ass_single(p, ["a:b", "c"])
            txt = p.read_text()
        self.assertIn("Dialogue: 0,0:00:00.00,0:00:05.00,Telem,,0,0,0,,a:b\\Nc",
                      txt)


class TestTelemetryLines(unittest.TestCase):
    def _auth(self):
        ev = {n: {"occurred_at": 0.5, "confirmed_at": 0.5,
                  "sample_index": 3999, "confirmed_sample_index": 3999}
              for n in R.EVENT_ORDER}
        return {"result": {"EVENTS": ev}}

    def _tel(self, t):
        return {"t": t, "mode": "PRELANDING|PRL_X", "com_z": 1.0,
                "com_vz": 0.0, "fz_N": 931.0, "fz_bw": 1.0, "fzL_N": 465.0,
                "fzR_N": 466.0, "supL": True, "supR": True, "u_max": 0.1}

    def test_pre_first_event_shows_none_yet(self):
        lines = R.telemetry_lines(self._auth(), 0, self._tel(0.000125))
        self.assertIn("LAST_EVENT: none yet", lines)
        self.assertFalse(any("nan" in s for s in lines))

    def test_post_confirmation_shows_last_event(self):
        lines = R.telemetry_lines(self._auth(), 3999, self._tel(0.5))
        self.assertTrue(any(s.startswith("LAST_EVENT: E12 stable_recovery")
                            for s in lines))
        self.assertTrue(any("OCCURS" in s for s in lines))


class TestGates(unittest.TestCase):
    def test_check_gate_shape(self):
        g = R.check_gate("X", True, "d")
        self.assertEqual(g, {"gate": "X", "pass": True, "detail": "d"})

    def test_fail_closed_exits_nonzero(self):
        with self.assertRaises(SystemExit) as cm:
            R.fail_closed("unit")
        self.assertNotEqual(cm.exception.code, 0)


class TestContainment(unittest.TestCase):
    def _scene(self):
        # Black clear color on top, neutral-grey floor below (frozen scene).
        frame = np.zeros((36, 64, 3), dtype=np.uint8)
        frame[18:] = [63, 66, 72]
        return frame

    def test_empty_frame_reports_no_foreground(self):
        cc = R.frame_containment(self._scene())
        self.assertLess(cc["fg_frac"], 1e-6)
        self.assertFalse(cc["touch_left"])

    def test_center_blob_detected_not_clipped(self):
        frame = self._scene()
        frame[10:20, 28:36] = [200, 100, 50]  # orange bar-like
        cc = R.frame_containment(frame)
        self.assertGreater(cc["fg_frac"], 0.01)
        self.assertFalse(cc["touch_left"])
        self.assertFalse(cc["touch_top"])

    def test_edge_blob_flags_clip(self):
        frame = self._scene()
        frame[2:6, 0:2] = [200, 100, 50]
        cc = R.frame_containment(frame)
        self.assertTrue(cc["touch_left"])
        self.assertTrue(cc["touch_top"])

    def test_floor_gradient_not_foreground(self):
        # Distant floor darkens neutrally; must not count as athlete.
        frame = self._scene()
        frame[8:18] = [18, 19, 21]
        cc = R.frame_containment(frame)
        self.assertLess(cc["fg_frac"], 1e-6)

    def test_floor_presence(self):
        self.assertGreater(R.floor_presence(self._scene()), 0.3)


class TestNoControllerImport(unittest.TestCase):
    def test_harness_source_imports(self):
        src = (REPO / "tools" / "res79_smoke_replay.py").read_text()
        for mod in ("res72_integration", "balance_capture", "stable_recovery",
                    "terminal_capture", "full_closure", "canonical_runtime"):
            self.assertNotIn(f"import {mod}", src)
            self.assertNotIn(f"from loaded_cmj.v2.{mod}", src)
            self.assertNotIn(f"from loaded_cmj.v2 import {mod}", src)

    def test_no_mj_step_on_viz_path(self):
        # The only stepping call site must live in replay_extract.
        src = (REPO / "tools" / "res79_smoke_replay.py").read_text()
        self.assertEqual(src.count("mujoco.mj_step"), 1)
        self.assertIn("def replay_extract", src)


if __name__ == "__main__":
    unittest.main()
