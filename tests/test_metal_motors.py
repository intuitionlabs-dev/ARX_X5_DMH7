"""Exercise the H7 application with the production DM driver on a host compiler."""
import ctypes as C
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

from metal_arm_control.can_protocol import decode_feedback
from metal_arm_control.config import MOTOR_PMAX, MOTOR_VMAX, MOTOR_TMAX
from metal_arm_control.usb2can import CANFrame

ROOT = Path(__file__).resolve().parents[1]
Float2, Float3, Float12 = (C.c_float*n for n in (2,3,12))
Bytes8 = C.c_uint8*8


@unittest.skipUnless(shutil.which('cc'), 'host C compiler required')
class MetalMotorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='metal-h7-motors-')
        cls.addClassCleanup(cls.tmp.cleanup)
        includes = ['App/inc','Bsp/inc','Core/Inc','Drivers/CMSIS/Include',
                    'Drivers/CMSIS/Device/ST/STM32H7xx/Include',
                    'Drivers/STM32H7xx_HAL_Driver/Inc','libs/gravity_compensation/c',
                    'libs/move_plan/curves','libs/move_plan/move_plan']
        includes += [str(p.relative_to(ROOT)) for p in (ROOT/'libs').iterdir() if p.is_dir()]
        sources = ['tests/c_metal_motors.c','libs/common/common.c','libs/cvector/cvector.c',
                   'libs/monitor/monitor.c','libs/dm_motor/dm_mit_control.c',
                   'libs/gravity_compensation/c/metal_dynamics.c']
        sources += [str(p.relative_to(ROOT)) for p in (ROOT/'libs/controller').glob('*.c')]
        models = str(Path(cls.tmp.name)/'models')
        subprocess.run(['python3',str(ROOT/'libs/gravity_compensation/c/generate_arm_model.py'),
                        '--output-dir',models],check=True)
        includes.append(models)
        sources += ['App/arm_profile.c','App/arm_board.c','App/arm_motor.c',
                    'App/profiles/metal.c','App/profiles/arx_x5.c','App/arm_initialization.c',
                    'App/arm_protocol.c','libs/gravity_compensation/c/arm_dynamics.c',
                    'libs/encos_motor/ENCOS_motor.c','libs/encos_motor/encos_protocol.c',
                    'libs/encos_motor/encos_mixed_control.c']
        library = str(Path(cls.tmp.name)/'motors.so')
        result = subprocess.run(['cc','-std=gnu11','-O2','-shared','-fPIC',
            '-fvisibility=hidden','-ffunction-sections','-fdata-sections',
            '-Werror=incompatible-pointer-types','-Wno-pointer-to-int-cast','-Wno-int-to-pointer-cast',
            '-DSTM32H723xx','-DUSE_HAL_DRIVER','-DUSE_PWR_LDO_SUPPLY',
            *['-I'+str(ROOT/p) for p in includes], *[str(ROOT/p) for p in sources],
            '-Wl,--gc-sections','-Wl,-z,defs','-lm','-o',library],capture_output=True,text=True)
        if result.returncode:
            raise RuntimeError(result.stdout+result.stderr)
        cls.lib = C.CDLL(library)
        fp, bp, up = C.POINTER(C.c_float), C.POINTER(C.c_uint8), C.POINTER(C.c_uint)
        for name,args in dict(metal_config=[C.c_uint,C.c_uint,fp],
            metal_feedback=[C.c_uint,C.c_uint,bp], metal_position_speed=[C.c_uint,C.c_uint,fp],
            metal_encode=[C.c_uint,C.c_uint,fp,C.c_int,bp], metal_rates=[C.c_uint,up],
            metal_mode=[C.c_uint,C.c_uint,fp], metal_zero=[C.c_uint,C.c_uint,up]).items():
            getattr(cls.lib,name).argtypes = args
        cls.lib.metal_fixture_init()

    def test_created_motors_use_metal_parameters(self):
        for bus in range(2):
            for i in range(7):
                buf = Float12(); self.lib.metal_config(bus,i,buf)
                actual = list(buf)
                expected = {0: i+1, 1: 0x11+i, 2: MOTOR_PMAX[i], 3: MOTOR_VMAX[i],
                            4: MOTOR_TMAX[i], 5: 500, 6: 5,
                            9: (0x55,0x11,0x22,0x22,0x44,0x88,0x88)[i]}
                for index, want in expected.items():
                    self.assertAlmostEqual(actual[index], want, places=6)

    def test_feedback_endpoints_and_two_bus_routing(self):
        for bus in range(2):
            for i in range(7):
                for raw in (0,1,32767,65535):
                    data = bytes((0x10+i+1,raw>>8,raw&255,0xff,0xff,0xff,31,37))
                    self.lib.metal_feedback(bus,0x11+i,Bytes8(*data))
                    actual = Float2(); self.lib.metal_position_speed(bus,i,actual)
                    expected = decode_feedback(bus,CANFrame(0x11+i,data),0)
                    self.assertAlmostEqual(actual[0],expected.position,places=5)
                    self.assertAlmostEqual(actual[1],expected.velocity,places=5)
        before = Float2(); self.lib.metal_position_speed(1,0,before)
        self.lib.metal_feedback(0,0x11,Bytes8(0x11,0,0,0,0,0,25,25))
        after = Float2(); self.lib.metal_position_speed(1,0,after)
        self.assertEqual(tuple(before),tuple(after))
        # Neither X5's shared ID=0 nor an inconsistent payload ID may update J1.
        before = Float2(); self.lib.metal_position_speed(0,0,before)
        for id,payload in ((0,0x11),(0x11,0x12)):
            self.lib.metal_feedback(0,id,Bytes8(payload,255,255,255,255,255,25,25))
            after = Float2(); self.lib.metal_position_speed(0,0,after)
            self.assertEqual(tuple(before),tuple(after))

    def test_position_ignores_accumulated_turns(self):
        # 累积圈数只参与位置换算；速度、力矩字段由闭环状态决定，此处不比对。
        rng = random.Random(1438)
        for bus in range(2):
            for i in range(7):
                for _ in range(50):
                    refs = Float3(*[rng.uniform(-.9*limit,.9*limit)
                                   for limit in (MOTOR_PMAX[i],MOTOR_VMAX[i],MOTOR_TMAX[i])])
                    positions = []
                    for turns in (-2,0,3):
                        actual = Bytes8(); self.lib.metal_encode(bus,i,refs,turns,actual)
                        positions.append(bytes(actual[:2]))
                    self.assertEqual(positions[0],positions[1])
                    self.assertEqual(positions[1],positions[2])

    def test_requested_rates_with_periodic_enable_and_frame_budget(self):
        counts = (C.c_uint*30)(); self.lib.metal_rates(1000,counts)
        for bus in range(2):
            self.assertEqual(list(counts[bus*15:bus*15+7]),[500,250,250,250,250,250,250])
            self.assertTrue(all(n>0 for n in counts[bus*15+7:bus*15+14]))
            self.assertLessEqual(counts[bus*15+14],3)

    def test_zero_uses_dm_commands_for_six_joints_and_explicit_override(self):
        for bus in range(2):
            for mask in (0,1<<6):
                out = (C.c_uint*11)(); self.lib.metal_zero(bus,mask,out)
                expected = mask or 0x3f
                self.assertEqual(list(out[:4]),[5,expected,expected,0])
                for i,result in enumerate(out[4:]):
                    self.assertEqual(result,2 if expected & (1<<i) else 5)

    def test_x5_mixed_driver_configuration_and_dispatch(self):
        counts = (C.c_uint*9)()
        self.lib.x5_create_and_rates.argtypes = [C.POINTER(C.c_uint)]
        self.assertEqual(self.lib.x5_create_and_rates(counts),1)
        for identifier,minimum in ((1,500),(2,250),(4,250),(5,250),(6,250),(7,250),(8,250)):
            self.assertGreaterEqual(counts[identifier],minimum)
        self.assertEqual(counts[3],0)
        self.lib.x5_motor_config.argtypes=[C.c_uint,C.POINTER(C.c_float)]
        for i,identifier in enumerate((1,2,4,5,6,7,8)):
            out=(C.c_float*8)()
            self.lib.x5_motor_config(i,out)
            self.assertEqual(out[0],identifier)
            self.assertEqual(out[1],identifier if i<3 else 0)
            self.assertEqual(out[7],i<3)
            self.assertEqual(out[2],12.5)
            self.assertEqual(out[3],18 if i<3 else 45)
        out=(C.c_float*2)()
        self.lib.x5_passive_enable.argtypes=[C.POINTER(C.c_float)]
        self.lib.x5_passive_enable(out)
        self.assertEqual(list(out),[0,0])


if __name__ == '__main__': unittest.main()
