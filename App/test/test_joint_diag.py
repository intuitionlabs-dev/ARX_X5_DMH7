"""Offline checks for the hardware experiment. Run with unittest discovery."""
import ctypes as C
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np
from joint_diag_data import (Benchmark, Clock, Info, Motor, Sample, analyze,
    choose_amplitude, clock_summary, integration_fit, lag_fit, preflight, trajectory, unwrap)

ROOT = Path(__file__).resolve().parents[2]


class DiagnosticTests(unittest.TestCase):
    def test_abi_matches_c_header_every_field(self):
        structures = (("H7_Diag_Info", Info), ("H7_Diag_Motor", Motor),
                      ("H7_Diag_Sample", Sample), ("H7_Diag_Clock", Clock),
                      ("H7_Diag_Benchmark", Benchmark))
        code = '#include <stddef.h>\n#include "h7_joint_diagnostics.h"\n'
        for name, cls in structures:
            code += f'_Static_assert(sizeof({name}) == {C.sizeof(cls)}, "{name} size");\n'
            for field, _ in cls._fields_:
                code += f'_Static_assert(offsetof({name}, {field}) == {getattr(cls, field).offset}, "{name}.{field}");\n'
        with tempfile.TemporaryDirectory() as temp:
            result = subprocess.run(["cc", "-std=c11", "-DTEST_JOINT_DIAGNOSTICS_ENABLE=1",
                "-I", str(ROOT/"App/test"), "-x", "c", "-c", "-o", str(Path(temp)/"abi.o"), "-"],
                input=code, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_trajectory_bounds_and_smooth_endpoints(self):
        for center, request in ((0, 20), (60, 45), (-90, 45)):
            amplitude = choose_amplitude(center, request, -115, 90)
            for kind in ("hold", "steps", "smooth"):
                path = [trajectory(kind, t, center, amplitude) for t in np.linspace(0, 3.2, 3201)]
                self.assertGreaterEqual(min(path), -100-1e-9)
                self.assertLessEqual(max(path), 75+1e-9)
                self.assertAlmostEqual(path[0], center)
                self.assertAlmostEqual(path[-1], center)
        for bad in (math.nan, math.inf, 0, -2, 46):
            with self.assertRaises(ValueError):
                choose_amplitude(0, bad, -115, 90)
        with self.assertRaises(ValueError):
            choose_amplitude(80, 20, -115, 90)
        for t in (.25, .95, 1.85, 2.65):
            dt = 1e-4
            values = [trajectory("smooth", t+x*dt, 0, 20) for x in (-1, 0, 1)]
            self.assertLess(abs((values[2]-values[0])/(2*dt)), .01)
            self.assertLess(abs((values[2]-2*values[1]+values[0])/(dt*dt)), .3)

    def test_clock_wrap_and_external_rate(self):
        for cpu, cmd, hal in ((480e6, 500, 1000), (367.2e6, 382.5, 765)):
            rows = []
            for k in range(101):
                t = k*.1
                rows.append(dict(before=100+t-.0002, after=100+t+.0002,
                    cycles=(int(cpu*t)+0xfffffff0) & 0xffffffff, hal_ms=round(t*hal),
                    rtos_tick=round(t*hal), cmd_count=round(t*cmd), deadline_misses=0))
            got = clock_summary(rows)
            self.assertAlmostEqual(got["cycles_per_s"], cpu, delta=1)
            self.assertAlmostEqual(got["cmd_count_per_s"], cmd, delta=.03)
        self.assertEqual(unwrap([0xfffffff0, 0x10, 0x40]), [0, 32, 80])

    def test_integral_fit_detects_reference_scale_mismatch(self):
        t = np.linspace(0, 4, 2001)
        v = 200*np.cos(2*math.pi*t)
        for scale in (1, .765):
            q = 13 + scale*200/(2*math.pi)*np.sin(2*math.pi*t)
            got = integration_fit(t, q, v)
            self.assertAlmostEqual(got["scale"], scale, delta=1e-4)
            self.assertLess(got["rms_deg"], .001)

    def test_phase_sign_and_amplitude_are_separate(self):
        t = np.arange(0, 4, .002)
        f = lambda x: 30*np.sin(2*math.pi*x)+8*np.sin(5*math.pi*x)
        for lag in (-.054, .046):
            got = lag_fit(t, f(t), t, .75*f(t-lag)+5)
            self.assertAlmostEqual(got["lag_ms"], lag*1000, delta=1.1)
            self.assertAlmostEqual(got["gain"], .75, delta=.01)
        self.assertFalse(lag_fit(t, np.zeros_like(t), t, np.zeros_like(t))["informative"])

    def test_real_fp32_solver_preflight(self):
        library = Path(os.environ.get("RUCKIG_DIAG_LIB", ROOT/"build/joint-diag-native/libruckig_diag.so"))
        if not library.exists():
            self.skipTest("build native Ruckig shared library per handoff")
        info = dict(period_s=.002, max_velocity=400, max_acceleration=2500, max_jerk=10000,
                    minimum_deg=[-115], maximum_deg=[90])
        for center, requested in ((0, 5), (0, 20), (0, 45), (60, 45), (-90, 45)):
            result = preflight(library, info, center, choose_amplitude(center, requested, -115, 90))
            self.assertEqual(len(result), 8)
            self.assertTrue(all(r["minimum"] > -105 and r["maximum"] < 80 for r in result))

    def test_analysis_uses_motor_timestamp_and_rejects_wrong_mode(self):
        rows = []
        for k in range(1600):
            t = k*.002
            q, v = 20*math.sin(t*5), 100*math.cos(t*5)
            tq = (k-1)*.002
            mq, mv = 20*math.sin((tq+.04)*5), 100*math.cos((tq+.04)*5)
            ref, ff = 20*math.sin(tq*5), 100*math.cos(tq*5)
            pid = 6*(ref-mq)
            rows.append(dict(seq=k+1, cycles=(2000000+int(t*480e6)) & 0xffffffff,
                flags=1+(4 << 4)+(1 << 9)+(1 << 13), wall_cycles=48000,
                ruckig_status=0, q=q, v=v, ruckig_q=q, ruckig_v=v, input_q=q, input_v=v,
                motor_seq=k+1, motor_cycles=(2000000+int(tq*480e6)) & 0xffffffff,
                motor_ref_pos=ref, motor_speed_comp=ff, motor_ref_speed=ff+pid,
                motor_fdb_pos=mq, motor_fdb_speed=mv, motor_pid_out=pid, motor_pid_error=ref-mq,
                motor_tx_speed_rad_s=(ff+pid)*math.pi/180,
                motor_flags=1+(3 << 8)+(1 << 16)+(1 << 18), motor_hal_ms=k*2,
                motor_pos_ms=k*2, motor_speed_ms=k*2, motor_rx_ms=k*2))
        info = dict(direction=1, reduction=1, zero_deg=0, period_s=.002, velocity_scale=1)
        result = analyze(rows, info, 480e6)
        self.assertTrue(result["valid_for_phase_conclusion"])
        self.assertAlmostEqual(result["position_feedback_lag"]["lag_ms"], -40, delta=1)
        self.assertLess(result["speed_sum_identity_max_error"], 1e-5)
        rows[20]["flags"] |= 1 << 10
        self.assertFalse(analyze(rows, info, 480e6)["valid_for_phase_conclusion"])
        rows[20]["flags"] &= ~(1 << 10)
        rows[20]["seq"] += 1
        self.assertFalse(analyze(rows, info, 480e6)["valid_for_phase_conclusion"])


if __name__ == "__main__":
    unittest.main()
