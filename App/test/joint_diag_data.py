"""H7 diagnostic ABI, bounded trajectories and offline signal analysis (no I/O)."""
import ctypes as C
import math

U = C.c_uint32
F = C.c_float


def fields(names, kind=U):
    return [(name, kind) for name in names.split()]


class Motor(C.LittleEndianStructure):
    _fields_ = fields("seq cycles hal_ms pos_ms speed_ms rx_ms flags cmd_seq") + fields(
        "ref_pos speed_comp ref_speed fdb_pos fdb_speed pid_out pid_error pid_d_error "
        "raw_pos_rad raw_speed_rad_s tx_pos_rad tx_speed_rad_s tx_torque mit_kp mit_kd", F)


class Sample(C.LittleEndianStructure):
    _fields_ = fields("seq cycles hal_ms rtos_tick wall_cycles flags") + [
        ("ruckig_status", C.c_int32)] + fields(
        "target q v a ruckig_time input_q input_v ruckig_q ruckig_v", F) + [("motor", Motor)]


class Info(C.LittleEndianStructure):
    _fields_ = fields("magic abi record_bytes capacity request state count generation error "
                     "guard_enable guard_fault guard_cycle_hz guard_timeout_ms") + fields("guard_margin_deg guard_error_deg", F) + fields(
        "core_hz hclk_hz pclk1_hz pclk2_hz hse_hz tick_hz systick_load tim23_psc tim23_arr "
        "rcc_cfgr rcc_d1cfgr rcc_d2cfgr rcc_pllckselr rcc_pll1divr cpuid ccr "
        "rc_online robot_state arm_mode tx_group control_type control_depth td_status pid_mode d_ahead") + fields(
        "period_s direction reduction zero_deg velocity_scale plan_velocity_scale "
        "max_velocity max_acceleration max_jerk pos_kp pos_ki pos_kd pos_output_max "
        "speed_kp speed_ki speed_kd mit_kp mit_kd position_max speed_max feedback_offset_rad", F) + [
        ("minimum_deg", F * 6), ("maximum_deg", F * 6),
        ("profile", C.c_char * 32), ("version", C.c_char * 64), ("build", C.c_char * 96)]


class Clock(C.LittleEndianStructure):
    _fields_ = fields("lock cycles hal_ms rtos_tick cmd_count deadline_misses wall_max_cycles hook_max_cycles")


class Cost(C.LittleEndianStructure):
    _fields_ = fields("count min_cycles max_cycles") + fields("mean_cycles", F)


class Benchmark(C.LittleEndianStructure):
    _fields_ = fields("completed status counter_overhead_cycles") + [("cost", Cost * 4), ("sink", F)]


def as_dict(value):
    if isinstance(value, C.Structure):
        return {name: as_dict(getattr(value, name)) for name, _ in value._fields_}
    if isinstance(value, C.Array):
        return [as_dict(item) for item in value]
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def flatten_sample(s):
    row = as_dict(s)
    motor = row.pop("motor")
    row.update({"motor_" + k: v for k, v in motor.items()})
    return row


def choose_amplitude(center, requested, low, high, margin=15.0):
    if not all(math.isfinite(x) for x in (center, requested, low, high, margin)):
        raise ValueError("nonfinite trajectory input")
    if requested <= 0 or requested > 45 or margin < 10 or low >= high:
        raise ValueError("amplitude must be (0,45] deg; margin >= 10 deg")
    amplitude = min(requested, center - low - margin, high - margin - center)
    if amplitude < 3:
        raise ValueError("JOINT1 is too close to a soft limit; no automatic repositioning")
    return amplitude


def trajectory(kind, t, center, amplitude):
    """2.4-second test, held start/end. Smooth segments have zero end v/a."""
    if kind == "hold":
        return center
    if kind == "steps":
        return center + (amplitude if .15 <= t < .90 else
                         -amplitude if .90 <= t < 1.65 else 0)
    if kind != "smooth":
        raise ValueError(kind)
    knots = ((0., 0.), (.15, 0.), (.80, amplitude), (1.55, -amplitude), (2.25, 0.), (2.4, 0.))
    for (ta, qa), (tb, qb) in zip(knots, knots[1:]):
        if ta <= t < tb:
            u = (t-ta)/(tb-ta)
            return center + qa + (qb-qa)*u**3*(10 + u*(-15 + 6*u))
    return center


class RuckigState(C.Structure):
    _fields_ = fields("position velocity acceleration", F)


class RuckigInput(C.Structure):
    _fields_ = [("current", RuckigState), ("target", RuckigState)] + fields(
        "max_velocity max_acceleration max_jerk", F)


class RuckigOutput(C.Structure):
    _fields_ = [("state", RuckigState)] + fields("time duration", F) + [("new_calculation", C.c_int)]


def preflight(library, info, center, amplitude, command_hz=100):
    """Use THIS checkout's real FP32 C solver; never let a path hit a limit.

    Try different real Cmd rates with the same fixed planner dt, including the
    previously suspected ~380 Hz. This is a reference-path check, not a plant
    stability proof. Hardware uses an additional feedback guard.
    """
    lib = C.CDLL(str(library))
    lib.ruckig_c_create.argtypes, lib.ruckig_c_create.restype = [F], C.c_void_p
    lib.ruckig_c_destroy.argtypes = [C.c_void_p]
    lib.ruckig_c_update.argtypes = [C.c_void_p, C.POINTER(RuckigInput), C.POINTER(RuckigOutput)]
    result = []
    for kind in ("steps", "smooth"):
        for rate in (250, 380, 500, 750):
            ctx = lib.ruckig_c_create(info["period_s"])
            if not ctx:
                raise RuntimeError("Ruckig allocation failed")
            inp = RuckigInput(RuckigState(center, 0, 0), RuckigState(center, 0, 0),
                              info["max_velocity"], info["max_acceleration"], info["max_jerk"])
            out = RuckigOutput()
            lo, hi = center, center
            try:
                for step in range(6 * rate):
                    t = math.floor(step / rate * command_hz) / command_hz
                    inp.target.position = trajectory(kind, t, center, amplitude)
                    status = lib.ruckig_c_update(ctx, C.byref(inp), C.byref(out))
                    if status < 0:
                        raise RuntimeError(f"Ruckig preflight failed: {kind} {rate} Hz, {status}")
                    q = out.state.position
                    if not math.isfinite(q) or not info["minimum_deg"][0]+10 < q < info["maximum_deg"][0]-10:
                        raise RuntimeError(f"reference outside 10 deg reserve: {q}")
                    lo, hi = min(lo, q), max(hi, q)
                    inp.current = out.state
                if abs(out.state.position-center) > .05 or abs(out.state.velocity) > .1:
                    raise RuntimeError("preflight did not settle")
            finally:
                lib.ruckig_c_destroy(ctx)
            result.append(dict(kind=kind, cmd_hz=rate, minimum=lo, maximum=hi))
    return result


def unwrap(values):
    out = [0]
    for a, b in zip(values, values[1:]):
        out.append(out[-1] + ((int(b)-int(a)) & 0xffffffff))
    return out


def clock_summary(rows):
    import numpy as np
    if len(rows) < 10:
        raise ValueError("at least ten external clock samples required")
    t = np.array([(r["before"]+r["after"])/2 for r in rows])
    t -= t[0]
    if t[-1] < 5 or np.max(np.diff(t)) >= 4:
        raise ValueError("need >=5 s calibration and <4 s gaps for DWT wrap")
    result = {"duration_s": float(t[-1]), "max_read_bracket_ms": max(
        r["after"]-r["before"] for r in rows)*1000,
        "timestamp_uncertainty_note": "read bracket plus up to one Cmd period (clock published at Cmd end)"}
    for key in ("cycles", "hal_ms", "rtos_tick", "cmd_count"):
        value = np.array(unwrap([r[key] for r in rows]), dtype=float)
        if key == "cycles" and np.any(np.diff(value)/np.diff(t) > 1e9):
            raise ValueError("DWT reset, debug halt, or bad clock read; recalibrate")
        slope, intercept = np.polyfit(t, value, 1)
        result[key+"_per_s"] = float(slope)
        result[key+"_fit_rms"] = float(np.sqrt(np.mean((value-(slope*t+intercept))**2)))
    result["deadline_misses"] = (rows[-1]["deadline_misses"]-rows[0]["deadline_misses"]) & 0xffffffff
    if not 50e6 < result["cycles_per_s"] < 1e9 or result["cmd_count_per_s"] < 10:
        raise ValueError("clock/command counter did not advance normally; do not move")
    return result


def integration_fit(t, q, v):
    import numpy as np
    integral = np.r_[0., np.cumsum(np.diff(t)*(v[:-1]+v[1:])*.5)]
    if np.ptp(integral) < 1:
        return {"informative": False}
    scale, offset = np.polyfit(integral, q, 1)
    return {"informative": True, "scale": float(scale),
            "rms_deg": float(np.sqrt(np.mean((q-offset-scale*integral)**2))),
            "position_span_deg": float(np.ptp(q))}


def lag_fit(ref_t, ref, fdb_t, fdb):
    """Positive lag means feedback occurs LATER: fdb(t) ~ ref(t-lag).

    Fixed overlap for every candidate; fit a gain and offset to avoid amplitude
    attenuation being misreported as a phase shift. Nonperiodic holds are not
    informative. This estimate includes sample/transport/plant effects.
    """
    import numpy as np
    lower, upper = max(ref_t[0], fdb_t[0])+.15, min(ref_t[-1], fdb_t[-1])-.15
    grid = np.arange(lower, upper, .002)
    if len(grid) < 30 or np.ptp(ref) < 1 or np.ptp(fdb) < 1:
        return {"informative": False}
    y = np.interp(grid, fdb_t, fdb)
    best = None
    for lag in np.arange(-.15, .1501, .001):
        x = np.interp(grid-lag, ref_t, ref)
        gain, offset = np.polyfit(x, y, 1)
        if gain <= 0:
            continue
        rms = float(np.sqrt(np.mean((y-gain*x-offset)**2)))
        if best is None or rms < best[0]:
            best = rms, float(lag), float(gain)
    if best is None:
        return {"informative": False}
    return dict(informative=True, lag_ms=best[1]*1000, gain=best[2], rms=best[0],
                at_search_edge=abs(best[1]) >= .149)


def analyze(rows, info, cycles_hz):
    import numpy as np
    if len(rows) < 100 or cycles_hz <= 0:
        raise ValueError("not enough samples or invalid clock")
    col = lambda key: np.array([r[key] for r in rows])
    t = np.array(unwrap(col("cycles"))) / cycles_hz
    if np.any(np.diff(t) > .1) or np.any(np.diff(t) <= 0):
        raise ValueError("capture contains halt/reset/large scheduling gap")
    seq = col("seq").astype(np.int64)
    flags = col("flags").astype(np.int64)
    # Normal, 0x16, no RC, no automatic plan, active successful Ruckig.
    valid = ((flags & 15) == 1) & (((flags >> 4) & 15) == 4) & ((flags & (1 << 9)) != 0)
    valid &= (flags & ((1 << 8) | (1 << 10) | (1 << 11) | (1 << 12))) == 0
    valid &= ((flags & (1 << 13)) != 0) & (col("ruckig_status") >= 0) & ((flags >> 16) == 0)
    motors = []
    for row in rows:
        if row["motor_seq"] and (not motors or row["motor_seq"] != motors[-1]["motor_seq"]):
            motors.append(row)
    if len(motors) < 30:
        raise ValueError("not enough unique motor calculations")
    mc = lambda key: np.array([r["motor_"+key] for r in motors])
    mt = np.array(unwrap(mc("cycles"))) / cycles_hz
    motor_offset = ((int(mc("cycles")[0])-int(col("cycles")[0])+(1 << 31)) & 0xffffffff)-(1 << 31)
    mt += motor_offset/cycles_hz
    to_joint = info["direction"]/info["reduction"]
    mq = (mc("fdb_pos")-info["zero_deg"])*to_joint
    mv = mc("fdb_speed")*to_joint
    identity = mc("ref_speed")-mc("speed_comp")-mc("pid_out")
    q, v = col("q"), col("v")
    result = dict(samples=len(rows), normal_position_samples=int(valid.sum()),
        contiguous_cmd_sequences=bool(np.all(np.diff(seq) == 1)),
        cmd_period_ms=dict(mean=float(np.mean(np.diff(t))*1000), p99=float(np.percentile(np.diff(t), 99)*1000),
                           maximum=float(np.max(np.diff(t))*1000)),
        cmd_wall_us=dict(mean=float(np.mean(col("wall_cycles"))/cycles_hz*1e6),
                         maximum=float(np.max(col("wall_cycles"))/cycles_hz*1e6)),
        motor_unique_samples=len(motors), motor_sequence_gaps=int(np.sum(np.diff(mc("seq")) != 1)),
        motor_period_ms=float(np.mean(np.diff(mt))*1000),
        motor_average_period_ms_from_counter=float((mt[-1]-mt[0])*1000/(int(mc("seq")[-1])-int(mc("seq")[0]))),
        reference_integral=integration_fit(t, q, v),
        reference_integral_nominal_2ms=integration_fit((seq-seq[0])*info["period_s"], q, v),
        motor_reference_integral=integration_fit(mt, (mc("ref_pos")-info["zero_deg"])*to_joint,
            mc("speed_comp")*to_joint/info["velocity_scale"]) if info["velocity_scale"] else {"informative": False},
        feedback_integral=integration_fit(mt, mq, mv),
        position_feedback_lag=lag_fit(t, q, mt, mq),
        speed_feedback_lag_vs_ff=lag_fit(mt, mc("speed_comp"), mt, mc("fdb_speed")),
        speed_feedback_lag_vs_command=lag_fit(mt, mc("ref_speed"), mt, mc("fdb_speed")),
        speed_sum_identity_max_error=float(np.max(np.abs(identity))),
        position_error_identity_max_error=float(np.max(np.abs(mc("pid_error")-(mc("ref_pos")-mc("fdb_pos"))))),
        post_vs_ruckig_q_max=float(np.max(np.abs(q-col("ruckig_q")))),
        post_vs_ruckig_v_max=float(np.max(np.abs(v-col("ruckig_v")))),
        ruckig_state_continuity_q_max=float(np.max(np.abs(col("input_q")[1:]-col("ruckig_q")[:-1]))),
        ruckig_state_continuity_v_max=float(np.max(np.abs(col("input_v")[1:]-col("ruckig_v")[:-1]))),
        tx_speed_conversion_error=float(np.max(np.abs(mc("tx_speed_rad_s")*180/math.pi-mc("ref_speed")))),
        rx_types=sorted({int(f) & 255 for f in mc("flags")}),
        raw_controller_matching_fraction=float(np.mean((mc("flags").astype(np.int64) & (1 << 16)) != 0)),
        new_calculation_fraction=float(np.mean((flags & (1 << 14)) != 0)))
    mf = mc("flags").astype(np.int64)
    for field in ("pos_ms", "speed_ms", "rx_ms"):
        ages = ((mc("hal_ms").astype(np.int64)-mc(field).astype(np.int64)) & 0xffffffff)
        matched = ages[(mf & (1 << 16)) != 0]
        result[field+"_age_ms"] = None if not len(matched) else dict(
            mean=float(np.mean(matched)), maximum=int(np.max(matched)))
    motor_valid = ((mf & (1 << 18)) != 0) & ((mf & (3 << 8)) == (3 << 8)) & ((mf >> 24) == 0)
    result["motor_enabled_valid_fraction"] = float(np.mean(motor_valid))
    result["valid_for_phase_conclusion"] = bool(valid.all() and motor_valid.all()
                                               and result["contiguous_cmd_sequences"])
    result["phase_sign"] = "positive = feedback lags; negative = feedback leads"
    return result
