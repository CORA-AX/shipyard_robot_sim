"""Bounded JSON messages for one local controller, one observation/action per step.

No simulator dependencies. The simulator owns time; this channel never advances it.
"""
import json
import math
import socket
import time
import uuid

PROTOCOL = 2
MAX_MESSAGE = 16384


class ProtocolError(ValueError):
    pass


def encode(message):
    data = json.dumps(message, allow_nan=False, separators=(',', ':')).encode() + b'\n'
    if len(data) > MAX_MESSAGE:
        raise ProtocolError('Message too large.')
    return data


def decode(data):
    try:
        message = json.loads(data)
    except (ValueError, UnicodeError) as exc:
        raise ProtocolError('Invalid JSON message.') from exc
    if not isinstance(message, dict):
        raise ProtocolError('Expected a JSON object.')
    return message


def vector(values, size):
    try:
        valid = (isinstance(values, list) and len(values) == size
                 and all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 3.402823e38
                         for v in values))
    except OverflowError:
        valid = False
    if not valid:
        raise ProtocolError(f'Expected {size} finite numeric values.')
    return values


class ControlServer:
    """Nonblocking local server. poll() accepts only the current request's action."""
    def __init__(self, port, robot_id, *, timeout=2.0, on_configure=None):
        if timeout <= 0:
            raise ValueError('Control timeout must be positive.')
        self.on_configure = on_configure
        self.configured = False
        self.robot_id = robot_id
        self.timeout = timeout
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.listener.bind(('127.0.0.1', port))
            self.listener.listen(2)
            self.listener.setblocking(False)
        except BaseException:
            self.listener.close()
            raise
        self.port = self.listener.getsockname()[1]
        self.peer = None
        self.pending = None
        self.buffer = b''
        self.output = b''
        self.connections = 0
        self.disconnects = 0
        self.rejections = 0
        self.last_reason = None

    def disconnect(self, reason):
        if self.peer is not None:
            self.peer.close()
            self.disconnects += 1
        self.peer = None
        self.pending = None
        self.buffer = self.output = b''
        self.configured = False
        self.last_reason = reason

    def _flush(self):
        if self.output:
            try:
                sent = self.peer.send(self.output)
            except BlockingIOError:
                return
            if sent == 0:
                raise ConnectionError('Connection closed during send.')
            self.output = self.output[sent:]

    def poll(self, observation):
        """Return an action list or None; never wait for a socket operation."""
        try:
            new, _ = self.listener.accept()
        except BlockingIOError:
            new = None
        if new is not None:
            if self.peer is not None:
                new.close()  # One controller owns this robot at a time.
            else:
                self.peer = new
                new.setblocking(False)
                new.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.session = uuid.uuid4().hex
                self.connections += 1
        if self.peer is None:
            return None
        try:
            if self.pending is None:
                self.pending = {**observation, 'protocol': PROTOCOL, 'robot_id': self.robot_id,
                                'session': self.session,
                                'type': 'observation' if self.configured else 'hello'}
                if not self.configured:
                    self.pending.pop('observation', None)
                self.output = encode(self.pending)
                self.deadline = time.monotonic() + self.timeout
            if time.monotonic() > self.deadline:
                self.disconnect('action_timeout')
                return None
            self._flush()
            try:
                data = self.peer.recv(MAX_MESSAGE + 1)
            except BlockingIOError:
                return None
            if not data:
                self.disconnect('controller_closed')
                return None
            self.buffer += data
            if len(self.buffer) > MAX_MESSAGE:
                raise ProtocolError('Message too large.')
            if b'\n' not in self.buffer:
                return None
            line, self.buffer = self.buffer.split(b'\n', 1)
            message = decode(line)
            if self.buffer or self.output:
                raise ProtocolError('Only one action per delivered observation is allowed.')
            for key in ('protocol', 'robot_id', 'session', 'step', 'episode'):
                if message.get(key) != self.pending[key]:
                    raise ProtocolError(f'Mismatched {key}.')
            if not self.configured:
                if message.get('type') != 'configure' or not isinstance(message.get('command'), dict):
                    raise ProtocolError('Expected controller configuration before actions.')
                if self.on_configure is not None:
                    self.on_configure(message['command'])
                self.configured = True
                self.pending = None
                return None  # Caller recomputes observations before the next poll.
            if message.get('type') == 'pause':
                self.disconnect('controller_paused')
                return None
            if message.get('type') != 'action':
                raise ProtocolError('Expected action or pause.')
            actions = vector(message.get('actions'), self.pending['action_size'])
            self.pending = None
            return actions
        except (OSError, ConnectionError, ProtocolError, ValueError) as exc:
            self.rejections += 1
            self.disconnect(str(exc))
            return None

    def finish(self, message):
        """Send a bounded final acknowledgement after the final applied action."""
        if self.peer is not None:
            try:
                self.peer.settimeout(0.5)
                self.peer.sendall(encode({**message, 'type': 'done', 'protocol': PROTOCOL,
                                          'robot_id': self.robot_id, 'session': self.session}))
            except OSError:
                pass

    def close(self):
        self.disconnect('viewer_closed')
        self.listener.close()


def receive(stream):
    data = stream.readline(MAX_MESSAGE + 1)
    if not data:
        raise ConnectionError('Viewer disconnected.')
    if len(data) > MAX_MESSAGE or not data.endswith(b'\n'):
        raise ProtocolError('Truncated or oversized message.')
    return decode(data)
