"""Control ownership, framing and stale-action checks without Isaac Sim."""
import socket
import sys
from pathlib import Path
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control.control_channel import ControlServer, ProtocolError, encode, receive, vector


class ChannelChecks(unittest.TestCase):
    def setUp(self):
        self.commands = []
        self.server = ControlServer(0, 'go2', timeout=2, on_configure=self.commands.append)
        self.client = socket.create_connection(('127.0.0.1', self.server.port), timeout=1)
        self.client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.stream = self.client.makefile('rb')
        self.obs = {'step': 0, 'episode': 0, 'action_size': 12, 'observation': [0.0] * 48}
        self.assertIsNone(self.server.poll(self.obs))
        self.request = receive(self.stream)
        self.assertEqual(self.request['type'], 'hello')
        self.assertNotIn('observation', self.request)
        self.client.sendall(self.reply(type='configure', command={'vx': 0.4}))
        self.assertIsNone(self.server.poll(self.obs))
        self.assertTrue(self.server.configured)
        self.assertIsNone(self.server.poll(self.obs))
        self.request = receive(self.stream)

    def tearDown(self):
        self.stream.close()
        self.client.close()
        self.server.close()

    def reply(self, **updates):
        result = {k: self.request[k] for k in ('protocol', 'robot_id', 'session', 'step', 'episode')}
        result.update(type='action', actions=[0.1] * 12)
        result.update(updates)
        return encode(result)

    def test_fragmented_action_applies_once(self):
        data = self.reply()
        self.client.sendall(data[:8])
        self.assertIsNone(self.server.poll(self.obs))
        self.client.sendall(data[8:])
        self.assertEqual(self.server.poll(self.obs), [0.1] * 12)
        self.assertIsNone(self.server.poll({**self.obs, 'step': 1}))
        self.assertEqual(receive(self.stream)['step'], 1)

    def test_stale_action_detaches_without_applying(self):
        self.client.sendall(self.reply(step=99))
        self.assertIsNone(self.server.poll(self.obs))
        self.assertIsNone(self.server.peer)
        self.assertEqual(self.server.rejections, 1)

    def test_wrong_robot_detaches(self):
        self.client.sendall(self.reply(robot_id='other'))
        self.assertIsNone(self.server.poll(self.obs))
        self.assertIsNone(self.server.peer)

    def test_pause_and_reconnect_has_new_session_same_step(self):
        self.client.sendall(self.reply(type='pause'))
        self.assertIsNone(self.server.poll(self.obs))
        self.assertEqual(self.server.last_reason, 'controller_paused')
        with socket.create_connection(('127.0.0.1', self.server.port), timeout=1) as second:
            self.assertIsNone(self.server.poll(self.obs))
            with second.makefile('rb') as stream:
                request = receive(stream)
            self.assertEqual(request['step'], 0)
            self.assertNotEqual(request['session'], self.request['session'])
            self.assertEqual(request['type'], 'hello')
            self.assertFalse(self.server.configured)
            self.assertNotIn('observation', request)
            second.sendall(encode({**{k: request[k] for k in
                ('protocol', 'robot_id', 'session', 'step', 'episode')},
                'type': 'configure', 'command': {'vx': 0.6}}))
            self.assertIsNone(self.server.poll(self.obs))
            self.assertEqual(self.commands, [{'vx': 0.4}, {'vx': 0.6}])
            self.assertTrue(self.server.configured)

    def test_timeout_and_duplicate_controller(self):
        with socket.create_connection(('127.0.0.1', self.server.port), timeout=1) as second:
            self.assertIsNone(self.server.poll(self.obs))
            self.assertEqual(second.recv(1), b'')
        with patch('control.control_channel.time.monotonic', return_value=self.server.deadline + 1):
            self.assertIsNone(self.server.poll(self.obs))
        self.assertIsNone(self.server.peer)
        self.assertEqual(self.server.last_reason, 'action_timeout')

    def test_invalid_actions_cannot_advance_physics(self):
        for values in ([0.0] * 11, [True] * 12, [float('nan')] * 12, ['1'] * 12,
                       [1e100] * 12, [10 ** 1000] * 12):
            with self.assertRaises(ProtocolError):
                vector(values, 12)
        self.client.sendall(b'{"type":"action","actions":[NaN]}\n')
        self.assertIsNone(self.server.poll(self.obs))
        self.assertEqual(self.server.rejections, 1)

    def test_actions_before_configuration_are_rejected(self):
        self.server.configured = False
        self.server.pending = None
        self.server.poll(self.obs)
        hello = receive(self.stream)
        self.client.sendall(encode({**{k: hello[k] for k in
            ('protocol', 'robot_id', 'session', 'step', 'episode')},
            'type': 'action', 'actions': [0.0] * 12}))
        self.assertIsNone(self.server.poll(self.obs))
        self.assertIsNone(self.server.peer)
        self.assertEqual(self.server.rejections, 1)

    def test_rejected_configuration_detaches_without_actions(self):
        def reject(command):
            raise ProtocolError('Invalid velocity')
        self.server.on_configure = reject
        self.server.configured = False
        self.server.pending = None
        self.server.poll(self.obs)
        hello = receive(self.stream)
        self.client.sendall(encode({**{k: hello[k] for k in
            ('protocol', 'robot_id', 'session', 'step', 'episode')},
            'type': 'configure', 'command': {'vx': 9.0}}))
        self.assertIsNone(self.server.poll(self.obs))
        self.assertIsNone(self.server.peer)
        self.assertFalse(self.server.configured)
        self.assertEqual(self.server.rejections, 1)
