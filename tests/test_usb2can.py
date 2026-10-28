"""传输配置、后端隔离和进程配置快照的回归用例。"""

import json
from pathlib import Path
import pickle
import tempfile
import unittest
from unittest.mock import patch

from arx_x5_control.usb2can import CANFrame, TransportConfigError, TransportRegistry
from arx_x5_control import CANChannel, CANMotorIO, ProcessCANMotorIO
from arx_x5_control.can_protocol import CANFrame as MotorCANFrame


class ConfiguredTransport:
    def __init__(self, channel, *, bitrate, marker):
        self.channel, self.bitrate, self.marker = channel, bitrate, marker

    def send(self, frame):
        raise AssertionError("只读配置用例不得发送报文")

    def receive(self):
        return []

    def close(self):
        Path(self.marker).write_text(self.channel)


class RegistryTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.path = self.root/'transports.json'
        self.data = dict(version=1, transports=dict(vendor={
            'class': __name__+':ConfiguredTransport', 'channel_kind': 'name',
            'options': {'marker': str(self.root/'closed')},
        }))
        self.write()

    def write(self):
        self.path.write_text(json.dumps(self.data))

    def test_shared_frame_and_default_registry(self):
        self.assertIs(MotorCANFrame, CANFrame)
        with patch.dict('os.environ', {}, clear=True):
            registry = TransportRegistry()
        self.assertEqual(registry.names, ('slcan', 'socketcan'))

    def test_configured_backend_receives_options(self):
        registry = TransportRegistry(self.path)
        transport = registry.create('vendor', 'device-2', bitrate=1000000)
        self.assertEqual((transport.channel, transport.bitrate), ('device-2', 1000000))
        transport.close()
        self.assertEqual((self.root/'closed').read_text(), 'device-2')

    def test_explicit_configuration_and_snapshot(self):
        with patch.dict('os.environ', {'USB2CAN_CONFIG': str(self.root/'missing')}):
            registry = TransportRegistry(self.path)
        self.path.unlink()
        snapshot = pickle.loads(pickle.dumps(registry))
        self.assertEqual(snapshot.names, ('vendor',))
        snapshot.create('vendor', 'endpoint').close()

    def test_environment_configuration(self):
        with patch.dict('os.environ', {'USB2CAN_CONFIG': str(self.path)}):
            self.assertEqual(TransportRegistry().names, ('vendor',))

    def test_specification_cannot_change_registry(self):
        registry = TransportRegistry(self.path)
        registry.specification('vendor').options['marker'] = 'other'
        self.assertEqual(registry.specification('vendor').options['marker'], str(self.root/'closed'))

    def test_unknown_interface_and_missing_class(self):
        registry = TransportRegistry(self.path)
        with self.assertRaisesRegex(TransportConfigError, 'unknown transport'):
            registry.create('missing', 'device')
        self.data['transports']['vendor']['class'] = 'arx_x5_control.usb2can:MissingTransport'
        self.write()
        with self.assertRaisesRegex(TransportConfigError, 'cannot load transport'):
            TransportRegistry(self.path).create('vendor', 'device')

    def test_invalid_configuration_rejected(self):
        for content in ('[]', '{"version":true,"transports":{}}',
                        '{"version":1,"version":1,"transports":{}}',
                        '{"version":1,"transports":{"x":{"class":"arx_x5_control.usb2can:SocketCANTransport","channel_kind":"name","options":{"bitrate":1}}}}',
                        '{"version":1,"transports":{"x":{"class":"arx_x5_control.usb2can:SocketCANTransport","channel_kind":"name","options":{"timeout":NaN}}}}'):
            with self.subTest(content=content):
                self.path.write_text(content)
                with self.assertRaises(TransportConfigError):
                    TransportRegistry(self.path)

    def test_alias_paths_share_endpoint(self):
        real = self.root/'device'
        real.touch()
        alias = self.root/'alias'
        alias.symlink_to(real)
        profile = self.data['transports']['vendor']
        profile['channel_kind'] = 'path'
        self.data['transports']['other'] = dict(profile)
        self.write()
        with self.assertRaisesRegex(ValueError, 'unique'):
            CANMotorIO([CANChannel(0, str(real), 'vendor'), CANChannel(1, str(alias), 'other')],
                       transport_config=self.path)

    def test_spawn_uses_snapshot_after_configuration_removed(self):
        io = ProcessCANMotorIO([CANChannel(0, 'device-2', 'vendor')],
                               transport_config=self.path, read_only=True)
        self.path.unlink()
        with io:
            io.poll_feedback(0)
        self.assertEqual((self.root/'closed').read_text(), 'device-2')


if __name__ == '__main__':
    unittest.main()
