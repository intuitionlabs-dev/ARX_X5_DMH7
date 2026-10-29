"""验证配置轴数、机型差异及 URDF 动力学。"""
import ctypes as C
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
INCLUDES = ['App/inc','Bsp/inc','Core/Inc','Drivers/CMSIS/Include',
            'Drivers/CMSIS/Device/ST/STM32H7xx/Include','Drivers/STM32H7xx_HAL_Driver/Inc',
            'libs/gravity_compensation/c','libs/move_plan/curves','libs/move_plan/move_plan']
INCLUDES += [str(p.relative_to(ROOT)) for p in (ROOT/'libs').iterdir() if p.is_dir()]
FLAGS = ['-std=gnu11','-O2','-Werror=incompatible-pointer-types','-Wno-pointer-to-int-cast',
         '-Wno-int-to-pointer-cast','-DSTM32H723xx','-DUSE_HAL_DRIVER','-DUSE_PWR_LDO_SUPPLY']


class FrameworkTests(unittest.TestCase):
    @staticmethod
    def write_axis_header(folder, count):
        for source in (ROOT/'App/inc').glob('*.h'):
            shutil.copy(source,folder/source.name)
        text = (folder/'robot_def.h').read_text()
        begin = text.index('    JOINT1 = 0,')
        end = text.index('    ARM_JOINT_NUM,',begin)
        text = text[:begin]+''.join(f'    JOINT{i+1} = {i},\n' for i in range(count))+text[end:]
        (folder/'robot_def.h').write_text(text)

    def test_six_axis_core(self):
        sources = ['tests/c_arm_framework.c','App/gimbal_arm.c','App/gimbal_cmd.c',
                   'App/arm_profile.c','App/arm_protocol.c','App/arm_zero.c','App/arm_auto.c',
                   'App/arm_initialization.c','libs/gravity_compensation/c/arm_dynamics.c',
                   'libs/controller/adrc.c','libs/common/common.c','libs/monitor/monitor.c',
                   'libs/cvector/cvector.c','libs/move_plan/curves/quintic_spline.c',
                   'libs/move_plan/curves/linear_spline.c','libs/move_plan/curves/cubic_spline.c',
                   'libs/move_plan/move_plan/move_plan.c','libs/move_plan/move_plan/plan_group.c']
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            self.write_axis_header(folder,6)
            executable = folder/'framework'
            result = subprocess.run(['cc',*FLAGS,'-I'+temp,*['-I'+str(ROOT/p) for p in INCLUDES],
                *[str(ROOT/p) for p in sources],'-lm','-o',str(executable)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            result = subprocess.run([str(executable)])
            self.assertEqual(result.returncode,0,f'C 检查行号余数 {result.returncode}')

    def test_non_six_axis_configurations_fail_to_compile(self):
        for count in (0,1,4,5,7,12):
            with self.subTest(joints=count), tempfile.TemporaryDirectory() as temp:
                self.write_axis_header(Path(temp),count)
                result = subprocess.run(['cc','-std=c11','-fsyntax-only','-I'+temp,'-x','c','-'],
                    input='#include "robot_def.h"\n',capture_output=True,text=True)
                self.assertNotEqual(result.returncode,0)
                self.assertIn('当前 H7 固件要求 ARM_JOINT_NUM == 6',result.stderr)

    def test_urdf_fixed_links_preserve_gravity(self):
        from libs.gravity_compensation import GravityCompensator
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            links=''.join(f'<link name="{name}"><inertial><origin xyz="0.2 0.1 0.03" rpy="0.2 0.1 0"/>'
                '<mass value="1"/><inertia ixx="0.1" iyy="0.2" izz="0.3" ixy="0.01" ixz="0" iyz="0"/>'
                '</inertial></link>' for name in ('base_link','mount','one','tool','two','tip'))
            joints=''.join(f'<joint name="j{i}" type="{kind}"><parent link="{parent}"/>'
                f'<child link="{child}"/><origin xyz="0.1 0.2 0.3" rpy="0.1 0.2 0.3"/>'
                '<axis xyz="0 1 0"/></joint>' for i,(parent,child,kind) in enumerate((
                    ('base_link','mount','fixed'),('mount','one','revolute'),
                    ('one','tool','fixed'),('tool','two','continuous'),('two','tip','fixed'))))
            urdf=folder/'robot.urdf'
            urdf.write_text('<robot name="test">'+links+joints+'</robot>')
            subprocess.run(['python3',str(ROOT/'libs/gravity_compensation/c/generate_arm_model.py'),
                '--urdf',str(urdf),'--tip','tip','--symbol','test_links','--output',str(folder/'model.h')],check=True)
            wrapper=folder/'model.c'
            wrapper.write_text('''#include "model.h"
int compute(float* q,float* out) {
    const float zero[2]={0}; Arm_dynamics_work work[2];
    const Arm_dynamics_model model={.joint_count=2,.links=test_links};
    return Arm_Dynamics_Compute(&model,q,zero,zero,0,1,out,work,2);
}''')
            library=folder/'model.so'
            subprocess.run(['cc','-shared','-fPIC','-O2','-I'+temp,
                '-I'+str(ROOT/'libs/gravity_compensation/c'),str(wrapper),
                str(ROOT/'libs/gravity_compensation/c/arm_dynamics.c'),'-lm','-o',str(library)],check=True)
            lib=C.CDLL(str(library))
            lib.compute.argtypes=[C.POINTER(C.c_float)]*2
            reference=GravityCompensator.from_urdf(urdf,tip='tip')
            rng=random.Random(145)
            array=C.c_float*2
            for _ in range(100):
                q=array(rng.uniform(-3,3),rng.uniform(-3,3));out=array()
                self.assertEqual(lib.compute(q,out),0)
                for actual,expected in zip(out,reference.raw_torques(tuple(q))):
                    self.assertAlmostEqual(actual,expected,delta=1e-5)

    def test_generic_dynamics_matches_existing_models(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            subprocess.run(['python3',str(ROOT/'libs/gravity_compensation/c/generate_arm_model.py'),
                            '--output-dir',temp],check=True)
            wrapper = folder/'dynamics.c'
            wrapper.write_text('''
#include "arm_dynamics.h"
#include "metal_model.h"
#include "x5_model.h"
int compute(int metal, int scaled, int gravity_only, float* q, float* qd, float* qdd, float* out) {
    const float gs_m[]={1.2f,1.15f,1.1f,1.15f,1,1}, gs_x[]={.8f,.8f,.8f,1.32f,1.32f,1.32f};
    const float cs[]={1.1f,1.15f,1,1,1.1f,1.1f}, friction[]={.15f,.3025f,.52f,.42f,.015f,.015f};
    const float limits[]={0,0,.8f,1.2f,0,0};
    const Arm_dynamics_model model={6,metal?metal_links:x5_links,
        scaled?(metal?gs_m:gs_x):0,scaled&&metal?cs:0,scaled&&metal?friction:0,scaled&&metal?limits:0};
    Arm_dynamics_work work[6];
    return Arm_Dynamics_Compute(&model,q,qd,qdd,0,gravity_only,out,work,6);
}
''')
            library = folder/'dynamics.so'
            sources=['arm_dynamics.c','metal_dynamics.c','x5_dynamics.c','x5_gravity.c']
            subprocess.run(['cc','-shared','-fPIC','-O2','-I'+temp,
                '-I'+str(ROOT/'libs/gravity_compensation/c'),str(wrapper),
                *[str(ROOT/'libs/gravity_compensation/c'/p) for p in sources],'-lm','-o',str(library)],check=True)
            lib=C.CDLL(str(library))
            pointer=C.POINTER(C.c_float)
            lib.compute.argtypes=[C.c_int]*3+[pointer]*4
            lib.metal_dynamics_compute.argtypes=[C.c_int]+[pointer]*4+[C.c_int,pointer]
            lib.x5_dynamics_compute.argtypes=[C.c_int]+[pointer]*4+[C.c_int,pointer]
            rng=random.Random(4832)
            array=C.c_float*6
            for metal in (0,1):
                reference=lib.metal_dynamics_compute if metal else lib.x5_dynamics_compute
                for scaled in (0,1):
                    for gravity_only in (0,1):
                        for sample in range(100):
                            q=array(*[rng.uniform(-2,2) for _ in range(6)])
                            qd=array(*[0 if gravity_only else rng.uniform(-2,2) for _ in range(6)])
                            qdd=array(*[0 if gravity_only else rng.uniform(-3,3) for _ in range(6)])
                            actual,expected=array(),array()
                            self.assertEqual(lib.compute(metal,scaled,gravity_only,q,qd,qdd,actual),0)
                            self.assertEqual(reference(0,q,qd,qdd,None,scaled,expected),0)
                            for a,b in zip(actual,expected): self.assertAlmostEqual(a,b,delta=2e-5)


if __name__ == '__main__':
    unittest.main()
