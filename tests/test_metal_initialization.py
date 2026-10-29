"""Run H7 initialization and soft limits against the Metal SDK profile."""
import ctypes as C
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

from sdk.board_defs import metal_arm as profile

ROOT = Path(__file__).resolve().parents[1]
Float7, Float13, Float20 = (C.c_float*n for n in (7,13,20))
UInt10 = C.c_uint*10
WAIT_INIT, INITIALIZING, NORMAL, RESETTING = 2, 3, 4, 7
STOP, RUN, DAMPING = 0, 1, 2


@unittest.skipUnless(shutil.which('cc'), 'host C compiler required')
class MetalInitializationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='metal-h7-init-')
        cls.addClassCleanup(cls.tmp.cleanup)
        includes = ['App/inc','Bsp/inc','Core/Inc','Drivers/CMSIS/Include',
                    'Drivers/CMSIS/Device/ST/STM32H7xx/Include',
                    'Drivers/STM32H7xx_HAL_Driver/Inc','libs/gravity_compensation/c',
                    'libs/move_plan/curves','libs/move_plan/move_plan']
        includes += [str(p.relative_to(ROOT)) for p in (ROOT/'libs').iterdir() if p.is_dir()]
        sources = ['tests/c_metal_initialization.c','libs/common/common.c',
                   'libs/cvector/cvector.c','libs/monitor/monitor.c','libs/controller/adrc.c',
                   'libs/move_plan/move_plan/move_plan.c','libs/move_plan/move_plan/plan_group.c']
        sources += [f'libs/move_plan/curves/{kind}_spline.c' for kind in ('linear','cubic','quintic')]
        models = str(Path(cls.tmp.name)/'models')
        subprocess.run(['python3',str(ROOT/'libs/gravity_compensation/c/generate_arm_model.py'),
                        '--output-dir',models],check=True)
        includes.append(models)
        sources += ['App/arm_profile.c','App/arm_board.c','App/arm_motor.c',
                    'App/profiles/metal.c','App/profiles/arx_x5.c','App/arm_initialization.c',
                    'App/arm_protocol.c','libs/gravity_compensation/c/arm_dynamics.c',
                    'libs/encos_motor/ENCOS_motor.c','libs/encos_motor/encos_protocol.c',
                    'libs/encos_motor/encos_mixed_control.c']
        sources += ['App/arm_zero.c','libs/dm_motor/DM_motor.c','libs/dm_motor/dm_mit_control.c','libs/controller/controller.c','libs/controller/pid.c','libs/controller/lqr.c','libs/controller/smc.c','libs/controller/mrac.c','libs/controller/mpc.c']
        library = str(Path(cls.tmp.name)/'initialization.so')
        result = subprocess.run(['cc','-std=gnu11','-O2','-shared','-fPIC',
            '-fvisibility=hidden','-ffunction-sections','-fdata-sections',
            '-Werror=incompatible-pointer-types','-Wno-pointer-to-int-cast','-Wno-int-to-pointer-cast',
            '-DSTM32H723xx','-DUSE_HAL_DRIVER','-DUSE_PWR_LDO_SUPPLY',
            *['-I'+str(ROOT/p) for p in includes], *[str(ROOT/p) for p in sources],
            '-Wl,--gc-sections','-Wl,-z,defs','-lm','-o',library], capture_output=True,text=True)
        if result.returncode:
            raise RuntimeError(result.stdout+result.stderr)
        cls.lib = C.CDLL(library)
        fp, up = C.POINTER(C.c_float), C.POINTER(C.c_uint)
        for name,args in dict(init_fixture=[], init_feedback=[C.c_uint,fp],
            init_entry=[C.c_uint]*5, init_motor_lost=[C.c_uint]*2, init_tick=[C.c_uint], init_observe=[C.c_uint,up,fp],
            init_limits=[fp,C.c_uint,fp], init_reset=[C.c_uint,C.c_float]).items():
            getattr(cls.lib,name).argtypes = args
            getattr(cls.lib,name).restype = None

    def setUp(self):
        self.lib.init_fixture()

    def observe(self, bus):
        state, output = UInt10(), Float20()
        self.lib.init_observe(bus,state,output)
        return list(state), list(output)

    def feedback(self, bus, joints=(10,-10,10,-10,10,-10), gripper=35):
        self.lib.init_feedback(bus,Float7(*joints,gripper))

    def start(self, bus=0, state=WAIT_INIT, mode=STOP, lost=0, rc=0):
        self.lib.init_entry(bus,state,mode,lost,rc)

    def test_limits_match_h7_profile_at_boundaries_and_random_targets(self):
        rng = random.Random(1438)
        inputs = [profile.AXIS_MIN, profile.AXIS_MAX, (-999,)*7, (999,)*7]
        inputs += [tuple(rng.uniform(-400,400) for _ in range(7)) for _ in range(200)]
        for values in inputs:
            values = Float7(*values)
            out = Float13()
            self.lib.init_limits(values,0x7f,out)
            for i in range(7):
                # H7 J2 upper limit is calibrated to 4.8 deg; other axes use SDK limits.
                upper = 4.8 if i == 1 else profile.AXIS_MAX[i]
                expected = max(profile.AXIS_MIN[i], min(upper, values[i]))
                self.assertAlmostEqual(out[i],expected,delta=2e-5)
            self.assertEqual(list(out[:6]),list(out[7:]))
        self.lib.init_limits(Float7(*([999]*7)),0,out)
        self.assertEqual(list(out[:7]),[999]*7)

    def test_two_arms_hold_distinct_grippers_and_finish_without_zeroing(self):
        for bus, grip in enumerate((35,105)):
            self.feedback(bus,gripper=grip)
            self.start(bus)
            self.assertEqual(self.observe(bus)[0][7],0)  # stale serial target invalidated
        for tick in range(1005):
            self.lib.init_tick(0)
            for bus, grip in enumerate((35,105)):
                state,out = self.observe(bus)
                if tick <= 1000:
                    self.assertEqual(state[0],INITIALIZING)
                self.assertEqual(state[5:7],[0,0])  # no zero request / state transition
                self.assertEqual(out[19],0)  # no calibration push torque
                if state[0] == INITIALIZING:
                    self.assertEqual(out[18],grip)
                    self.assertEqual(state[4],0)
                    if tick == 500:
                        for q,start in zip(out[:6],(10,-10,10,-10,10,-10)):
                            self.assertAlmostEqual(q,start/2,delta=.001)
                    if tick >= 1000:
                        for q in out[:6]: self.assertAlmostEqual(q,0,delta=.001)
                else:
                    self.assertEqual(state[:2],[NORMAL,DAMPING])
                    self.assertEqual(state[4],1)
            if tick == 5:
                # Changing feedback must not change the latched gripper target.
                for bus, grip in enumerate((38,108)): self.feedback(bus,gripper=grip)
        for bus in range(2): self.assertEqual(self.observe(bus)[0][:2],[NORMAL,DAMPING])

    def test_reinitialization_captures_new_gripper_angle(self):
        for grip in (40,90):
            self.feedback(0,gripper=grip)
            self.start(0,WAIT_INIT if grip == 40 else NORMAL,STOP if grip == 40 else DAMPING)
            self.lib.init_tick(0)
            self.assertEqual(self.observe(0)[1][18],grip)
            self.feedback(0,gripper=grip+2)
            for _ in range(999):
                self.lib.init_tick(0)
                self.assertEqual(self.observe(0)[1][18],grip)
            for _ in range(5): self.lib.init_tick(0)
            self.assertEqual(self.observe(0)[0][:2],[NORMAL,DAMPING])

    def test_initialization_entry_guards(self):
        for axis in range(6):
            joints = [0]*6; joints[axis] = 20.01
            self.feedback(0,joints)
            self.start()
            self.assertEqual(self.observe(0)[0][0],WAIT_INIT)
        self.feedback(0,(0,)*6)
        for lost in [1<<i for i in range(7)]:
            self.start(lost=lost)
            self.assertEqual(self.observe(0)[0][0],WAIT_INIT)
        self.start(rc=1)
        self.assertEqual(self.observe(0)[0][0],WAIT_INIT)
        self.start(state=NORMAL,mode=RUN)
        self.assertEqual(self.observe(0)[0][0],NORMAL)
        self.feedback(0,(20,-20,20,-20,20,-20))
        self.start()
        self.assertEqual(self.observe(0)[0][0],INITIALIZING)

    def test_remote_abort_returns_to_entry_without_marking_initialized(self):
        for entry,mode in ((WAIT_INIT,STOP),(NORMAL,DAMPING)):
            self.lib.init_fixture()
            self.feedback(0)
            self.start(state=entry,mode=mode)
            for _ in range(20): self.lib.init_tick(0)
            self.lib.init_tick(1)
            state,_ = self.observe(0)
            self.assertEqual(state[0],entry)
            self.assertEqual(state[2:7],[0,0,0,0,0])
            self.assertEqual(state[9],1)

    def test_motor_loss_aborts_initialization_without_marking_gripper_ready(self):
        for index in range(7):
            for entry,mode in ((WAIT_INIT,STOP),(NORMAL,DAMPING)):
                self.lib.init_fixture()
                self.feedback(0)
                self.start(state=entry,mode=mode)
                for _ in range(20): self.lib.init_tick(0)
                self.lib.init_motor_lost(0,index)
                self.lib.init_tick(0)
                state,_ = self.observe(0)
                self.assertEqual(state[0],entry)
                self.assertEqual(state[2:7],[0,0,0,0,0])
                self.assertEqual(state[9],1)

    def test_runtime_reset_uses_zero_pose_without_init_tolerance(self):
        self.feedback(0,(60,-60,60,-60,60,-60),70)
        self.lib.init_reset(0,70)
        self.assertEqual(self.observe(0)[0][0],RESETTING)
        for tick in range(1005):
            self.lib.init_tick(0)
            state,out = self.observe(0)
            self.assertEqual(state[5:7],[0,0])
            if state[0] == RESETTING:
                self.assertEqual(out[18],70)
                if tick >= 1000:
                    for q in out[:6]: self.assertAlmostEqual(q,0,delta=.001)
        self.assertEqual(self.observe(0)[0][:2],[NORMAL,DAMPING])


if __name__ == '__main__':
    unittest.main()
