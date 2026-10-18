"""传输配置接口、权限处理和启动顺序的回归用例。"""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from arx_x5_control import CANChannel, CANMotorIO
from arx_x5_control.usb2can import SocketCANTransport


def result(code=0, stderr=''):
    return SimpleNamespace(returncode=code, stderr=stderr, stdout='')


class ConfigurationTests(unittest.TestCase):
    def transport(self):
        transport = SocketCANTransport.__new__(SocketCANTransport)
        transport._channel, transport._index = 'can0', 1
        transport._configured = False
        transport._socket = Mock()
        transport._open_socket = Mock(return_value=Mock())
        return transport

    def test_fixed_parameters_and_idempotence(self):
        transport = self.transport()
        with patch('socket.if_nametoindex', return_value=1), \
                patch('arx_x5_control.usb2can.socketcan.shutil.which', return_value='/usr/sbin/ip'), \
                patch('arx_x5_control.usb2can.socketcan.subprocess.run', return_value=result()) as run:
            transport.configure()
            transport.configure()
        self.assertEqual([c.args[0] for c in run.call_args_list], [
            ['/usr/sbin/ip', 'link', 'set', 'dev', 'can0', 'down'],
            ['/usr/sbin/ip', 'link', 'set', 'dev', 'can0', 'type', 'can',
             'bitrate', '1000000', 'listen-only', 'off'],
            ['/usr/sbin/ip', 'link', 'set', 'dev', 'can0', 'txqueuelen', '16'],
            ['/usr/sbin/ip', 'link', 'set', 'dev', 'can0', 'up'],
        ])
        self.assertTrue(transport._configured)
        transport._open_socket.assert_called_once()

    def test_permission_failure_uses_noninteractive_sudo(self):
        transport = self.transport()
        replies = [result(2, 'RTNETLINK answers: Operation not permitted'), result()]*4
        with patch('socket.if_nametoindex', return_value=1), \
                patch('arx_x5_control.usb2can.socketcan.shutil.which', return_value='/usr/sbin/ip'), \
                patch('arx_x5_control.usb2can.socketcan.os.geteuid', return_value=1000), \
                patch('arx_x5_control.usb2can.socketcan.subprocess.run', side_effect=replies) as run:
            transport.configure()
        self.assertEqual(run.call_count, 8)
        for call in run.call_args_list[1::2]:
            self.assertEqual(call.args[0][:3], ['sudo', '-n', '/usr/sbin/ip'])

    def test_failure_leaves_configuration_incomplete(self):
        transport = self.transport()
        with patch('socket.if_nametoindex', return_value=1), \
                patch('arx_x5_control.usb2can.socketcan.shutil.which', return_value='/usr/sbin/ip'), \
                patch('arx_x5_control.usb2can.socketcan.subprocess.run', side_effect=[
                    result(), result(2, 'Invalid argument'), result(),
                ]) as run:
            with self.assertRaisesRegex(OSError, 'Invalid argument'):
                transport.configure()
        self.assertFalse(transport._configured)
        self.assertEqual(run.call_args.args[0][-1], 'down')

    def test_all_channels_configured_before_start(self):
        calls = []

        class Backend:
            def __init__(self, config):
                self.arm = config.arm_id

            def configure(self):
                calls.append(('configure', self.arm))

            def start(self):
                calls.append(('start', self.arm))

            def receive(self):
                return []

            def send(self, frame):
                raise AssertionError('只读配置用例不得发送报文')

            def close(self):
                calls.append(('close', self.arm))

        with CANMotorIO([CANChannel(0, 'first'), CANChannel(1, 'second')],
                        read_only=True, transport_factory=Backend):
            self.assertEqual(calls, [('configure', 0), ('configure', 1), ('start', 0), ('start', 1)])
        self.assertEqual(calls[-2:], [('close', 0), ('close', 1)])

    def test_setup_failure_closes_every_opened_backend(self):
        closed = []

        class Backend:
            def __init__(self, config):
                self.arm = config.arm_id

            def configure(self):
                raise OSError('configuration failed')

            def close(self):
                closed.append(self.arm)

        io = CANMotorIO([CANChannel(0, 'first'), CANChannel(1, 'second')],
                        transport_factory=Backend)
        with self.assertRaisesRegex(OSError, 'configuration failed'):
            io.open()
        self.assertEqual(closed, [0, 1])
        self.assertFalse(io._opened)


if __name__ == '__main__':
    unittest.main()
